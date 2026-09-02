// DronePawn.cpp  (v2: dieu khien 2 chieu)

#include "DronePawn.h"
#include "CesiumGeoreference.h"
#include "Kismet/GameplayStatics.h"
#include "SocketSubsystem.h"
#include "GameFramework/PlayerController.h"
#include "Engine/Engine.h"

ADronePawn::ADronePawn()
{
	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.TickGroup = TG_PrePhysics;

	RootScene = CreateDefaultSubobject<USceneComponent>(TEXT("RootScene"));
	SetRootComponent(RootScene);

	BodyMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("BodyMesh"));
	BodyMesh->SetupAttachment(RootScene);
	BodyMesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	BodyMesh->SetMobility(EComponentMobility::Movable);

	AutoPossessPlayer = EAutoReceiveInput::Player0;
}

void ADronePawn::BeginPlay()
{
	Super::BeginPlay();

	if (!Georeference)
	{
		TArray<AActor*> Found;
		UGameplayStatics::GetAllActorsOfClass(GetWorld(), ACesiumGeoreference::StaticClass(), Found);
		if (Found.Num() > 0)
		{
			Georeference = Cast<ACesiumGeoreference>(Found[0]);
		}
		else
		{
			UE_LOG(LogTemp, Error, TEXT("[DronePawn] Khong tim thay CesiumGeoreference trong level!"));
		}
	}

	StartListening();

	if (bEnableControl)
	{
		StartSending();
	}
}

void ADronePawn::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	StopListening();
	StopSending();
	Super::EndPlay(EndPlayReason);
}

// ===========================================================================
//  NHAN TELEMETRY
// ===========================================================================

void ADronePawn::StartListening()
{
	FIPv4Endpoint Endpoint(FIPv4Address::Any, static_cast<uint16>(ListenPort));

	RecvSocket = FUdpSocketBuilder(TEXT("DroneTwinRecv"))
		.AsNonBlocking()
		.AsReusable()
		.BoundToEndpoint(Endpoint)
		.WithReceiveBufferSize(1 * 1024 * 1024);

	if (!RecvSocket)
	{
		UE_LOG(LogTemp, Error, TEXT("[DronePawn] Khong mo duoc UDP socket tren port %d"), ListenPort);
		return;
	}

	Receiver = new FUdpSocketReceiver(RecvSocket, FTimespan::FromMilliseconds(5), TEXT("DroneTwinRecvThread"));
	Receiver->OnDataReceived().BindUObject(this, &ADronePawn::OnUdpDataReceived);
	Receiver->Start();

	UE_LOG(LogTemp, Log, TEXT("[DronePawn] Dang lang nghe UDP tren port %d"), ListenPort);
}

void ADronePawn::StopListening()
{
	if (Receiver)
	{
		Receiver->Stop();
		delete Receiver;
		Receiver = nullptr;
	}

	if (RecvSocket)
	{
		RecvSocket->Close();
		ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->DestroySocket(RecvSocket);
		RecvSocket = nullptr;
	}
}

// LUU Y: chay tren thread rieng, khong duoc dung vao scene component o day.
void ADronePawn::OnUdpDataReceived(const FArrayReaderPtr& Data, const FIPv4Endpoint& Endpoint)
{
	if (!Data.IsValid() || Data->Num() < static_cast<int32>(sizeof(FDroneStatePacket)))
	{
		return;
	}

	FDroneStatePacket Packet;
	FMemory::Memcpy(&Packet, Data->GetData(), sizeof(FDroneStatePacket));

	if (Packet.Magic != DRONE_STATE_MAGIC)
	{
		return;
	}

	FScopeLock Lock(&StateMutex);

	const bool bNewer = (Packet.Seq > LastSeq) || (LastSeq - Packet.Seq > 1000);
	if (bHasPacket && !bNewer)
	{
		return;
	}

	LatestPacket = Packet;
	LastSeq = Packet.Seq;
	bHasPacket = true;
	LastPacketTime = FPlatformTime::Seconds();
}

// ===========================================================================
//  GUI DIEU KHIEN
// ===========================================================================

void ADronePawn::StartSending()
{
	SendSocket = FUdpSocketBuilder(TEXT("DroneTwinSend")).AsReusable();

	if (!SendSocket)
	{
		UE_LOG(LogTemp, Error, TEXT("[DronePawn] Khong mo duoc socket gui dieu khien"));
		return;
	}

	ISocketSubsystem* SS = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM);
	BridgeAddr = SS->CreateInternetAddr();

	bool bAddrValid = false;
	BridgeAddr->SetIp(*BridgeHost, bAddrValid);
	BridgeAddr->SetPort(BridgeControlPort);

	if (!bAddrValid)
	{
		UE_LOG(LogTemp, Error, TEXT("[DronePawn] Dia chi bridge khong hop le: %s"), *BridgeHost);
		StopSending();
		return;
	}

	UE_LOG(LogTemp, Log, TEXT("[DronePawn] Gui dieu khien toi %s:%d"), *BridgeHost, BridgeControlPort);
}

void ADronePawn::StopSending()
{
	if (SendSocket)
	{
		SendSocket->Close();
		ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->DestroySocket(SendSocket);
		SendSocket = nullptr;
	}
	BridgeAddr.Reset();
}

void ADronePawn::PollInput(float DeltaSeconds)
{
	APlayerController* PC = Cast<APlayerController>(GetController());
	if (!PC)
	{
		return;
	}

	// --- Doc ban phim thanh muc tieu -1..1 ---
	float TargetRoll = 0.f;
	float TargetPitch = 0.f;
	float TargetYaw = 0.f;

	if (PC->IsInputKeyDown(EKeys::Right)) TargetRoll += 1.f;
	if (PC->IsInputKeyDown(EKeys::Left))  TargetRoll -= 1.f;
	if (PC->IsInputKeyDown(EKeys::Up))    TargetPitch += 1.f;   // chui mui ve truoc
	if (PC->IsInputKeyDown(EKeys::Down))  TargetPitch -= 1.f;
	if (PC->IsInputKeyDown(EKeys::D))     TargetYaw += 1.f;
	if (PC->IsInputKeyDown(EKeys::A))     TargetYaw -= 1.f;

	// --- Tay cam (neu co) de len tren ban phim ---
	if (bUseGamepad)
	{
		float LX = 0.f, LY = 0.f, RX = 0.f, RY = 0.f;
		PC->GetInputAnalogStickState(EControllerAnalogStick::CAS_LeftStick, LX, LY);
		PC->GetInputAnalogStickState(EControllerAnalogStick::CAS_RightStick, RX, RY);

		const float Dead = 0.15f;
		if (FMath::Abs(RX) > Dead) TargetRoll = RX;
		if (FMath::Abs(RY) > Dead) TargetPitch = RY;
		if (FMath::Abs(LX) > Dead) TargetYaw = LX;
		if (FMath::Abs(LY) > Dead)
		{
			// Can trai doc = throttle tuyet doi, giong tay dieu khien that
			StickThrottle = FMath::Clamp((LY + 1.f) * 0.5f, 0.f, 1.f);
		}
	}

	// --- Can tu ve giua khi tha, giong lo xo cua tay dieu khien that ---
	auto MoveStick = [&](float Current, float Target) -> float
	{
		const float Rate = (FMath::Abs(Target) > KINDA_SMALL_NUMBER) ? StickRiseRate : StickReturnRate;
		return FMath::FInterpConstantTo(Current, Target, DeltaSeconds, Rate);
	};

	StickRoll = MoveStick(StickRoll, TargetRoll);
	StickPitch = MoveStick(StickPitch, TargetPitch);
	StickYaw = MoveStick(StickYaw, TargetYaw);

	// --- Throttle GIU nguyen khi tha phim (khong tu ve 0) ---
	if (PC->IsInputKeyDown(EKeys::W)) StickThrottle += ThrottleRate * DeltaSeconds;
	if (PC->IsInputKeyDown(EKeys::S)) StickThrottle -= ThrottleRate * DeltaSeconds;
	StickThrottle = FMath::Clamp(StickThrottle, 0.f, 1.f);

	// --- Arm / Disarm (chi kich hoat luc bam xuong, khong lap khi giu) ---
	const bool bArmDown = PC->IsInputKeyDown(EKeys::Enter) || PC->IsInputKeyDown(EKeys::Gamepad_Special_Right);
	if (bArmDown && !bArmKeyWasDown)
	{
		PendingArm = DRONE_ARM_ARM;
		PendingArmTimer = 0.5f;
		UE_LOG(LogTemp, Log, TEXT("[DronePawn] Yeu cau ARM"));
	}
	bArmKeyWasDown = bArmDown;

	const bool bDisarmDown = PC->IsInputKeyDown(EKeys::BackSpace) || PC->IsInputKeyDown(EKeys::Gamepad_Special_Left);
	if (bDisarmDown && !bDisarmKeyWasDown)
	{
		PendingArm = DRONE_ARM_DISARM;
		PendingArmTimer = 0.5f;
		UE_LOG(LogTemp, Log, TEXT("[DronePawn] Yeu cau DISARM"));
	}
	bDisarmKeyWasDown = bDisarmDown;

	// --- Doi mode bang phim so 1..6 ---
	static const FKey ModeKeys[6] = {
		EKeys::One, EKeys::Two, EKeys::Three, EKeys::Four, EKeys::Five, EKeys::Six
	};
	static const uint8 ModeValues[6] = {
		DRONE_MODE_STABILIZE, DRONE_MODE_ALT_HOLD, DRONE_MODE_LOITER,
		DRONE_MODE_GUIDED, DRONE_MODE_RTL, DRONE_MODE_LAND
	};

	for (int32 i = 0; i < 6; ++i)
	{
		const bool bDown = PC->IsInputKeyDown(ModeKeys[i]);
		if (bDown && !bModeKeyWasDown[i])
		{
			PendingMode = ModeValues[i];
			PendingModeTimer = 0.5f;
			UE_LOG(LogTemp, Log, TEXT("[DronePawn] Yeu cau doi mode -> %d"), PendingMode);
		}
		bModeKeyWasDown[i] = bDown;
	}

	// Lenh duoc giu trong 0.5s de chiu duoc mat goi UDP
	PendingArmTimer = FMath::Max(0.f, PendingArmTimer - DeltaSeconds);
	if (PendingArmTimer <= 0.f) PendingArm = DRONE_ARM_NONE;

	PendingModeTimer = FMath::Max(0.f, PendingModeTimer - DeltaSeconds);
	if (PendingModeTimer <= 0.f) PendingMode = DRONE_MODE_NONE;
}

void ADronePawn::SendControl()
{
	if (!SendSocket || !BridgeAddr.IsValid())
	{
		return;
	}

	FDroneControlPacket Packet;
	FMemory::Memzero(&Packet, sizeof(Packet));

	Packet.Magic = DRONE_CONTROL_MAGIC;
	Packet.Seq = ControlSeq++;
	Packet.Roll = StickRoll;
	Packet.Pitch = StickPitch;
	Packet.Yaw = StickYaw;
	Packet.Throttle = StickThrottle;
	Packet.ArmRequest = PendingArm;
	Packet.ModeRequest = PendingMode;

	int32 BytesSent = 0;
	SendSocket->SendTo(reinterpret_cast<const uint8*>(&Packet), sizeof(Packet), BytesSent, *BridgeAddr);
}

// ===========================================================================
//  AP DUNG TELEMETRY
// ===========================================================================

FVector ADronePawn::GeoToUnreal(double Lon, double Lat, double HeightMeters) const
{
	if (!Georeference)
	{
		return FVector::ZeroVector;
	}

	// Cesium for Unreal 1.24.0 — thu tu tham so la (Longitude, Latitude, Height).
	return Georeference->InaccurateTransformLongitudeLatitudeHeightToUnreal(
		FVector(Lon, Lat, HeightMeters));

	// Neu compile bao loi "no member named ...", ten ham da doi giua cac minor
	// version cua plugin. Mo CesiumGeoreference.h va tim ham co chu
	// "LongitudeLatitudeHeight...Unreal". Cac ten da tung dung:
	//   InaccurateTransformLongitudeLatitudeHeightToUe(const FVector&)
	//   TransformLongitudeLatitudeHeightPositionToUnreal(const FVector&)
	//   TransformLongitudeLatitudeHeightToUnreal(const glm::dvec3&)
}

void ADronePawn::ApplyTelemetry(float DeltaSeconds)
{
	FDroneStatePacket Packet;
	double PacketTime = 0.0;
	{
		FScopeLock Lock(&StateMutex);
		if (!bHasPacket)
		{
			bLinkAlive = false;
			return;
		}
		Packet = LatestPacket;
		PacketTime = LastPacketTime;
	}

	bLinkAlive = (FPlatformTime::Seconds() - PacketTime) < 2.0;

	const FVector TargetLocation = GeoToUnreal(Packet.Lon, Packet.Lat, Packet.AltMSL);
	const FRotator TargetRotation(Packet.Pitch, Packet.Yaw + YawOffsetDeg, Packet.Roll);

	if (LocationInterpSpeed > KINDA_SMALL_NUMBER)
	{
		SetActorLocation(FMath::VInterpTo(GetActorLocation(), TargetLocation, DeltaSeconds, LocationInterpSpeed));
	}
	else
	{
		SetActorLocation(TargetLocation);
	}

	if (RotationInterpSpeed > KINDA_SMALL_NUMBER)
	{
		SetActorRotation(FMath::RInterpTo(GetActorRotation(), TargetRotation, DeltaSeconds, RotationInterpSpeed));
	}
	else
	{
		SetActorRotation(TargetRotation);
	}

	// Canh quat (bo qua neu mesh da combine, mang rong)
	const float Spin = FMath::Clamp(Packet.Throttle, 0.f, 1.f) * MaxPropSpinDegPerSec;
	PropAngle = FMath::Fmod(PropAngle + Spin * DeltaSeconds, 360.f);

	for (int32 i = 0; i < Propellers.Num(); ++i)
	{
		if (!Propellers[i]) continue;
		const float Dir = (i % 2 == 0) ? 1.f : -1.f;
		Propellers[i]->SetRelativeRotation(FRotator(0.f, PropAngle * Dir, 0.f));
	}

	bIsArmed = (Packet.Armed != 0);
	AltitudeMSL = static_cast<float>(Packet.AltMSL);
	GroundSpeed = FMath::Sqrt(Packet.Vx * Packet.Vx + Packet.Vy * Packet.Vy);
	CurrentMode = Packet.Mode;
}

// ===========================================================================
//  OVERLAY
// ===========================================================================

static FString ModeName(uint8 Mode)
{
	switch (Mode)
	{
	case DRONE_MODE_STABILIZE: return TEXT("STABILIZE");
	case DRONE_MODE_ALT_HOLD:  return TEXT("ALT_HOLD");
	case DRONE_MODE_GUIDED:    return TEXT("GUIDED");
	case DRONE_MODE_LOITER:    return TEXT("LOITER");
	case DRONE_MODE_RTL:       return TEXT("RTL");
	case DRONE_MODE_LAND:      return TEXT("LAND");
	default:                   return FString::Printf(TEXT("MODE %d"), Mode);
	}
}

void ADronePawn::DrawOverlay()
{
	if (!GEngine)
	{
		return;
	}

	const FColor LinkColor = bLinkAlive ? FColor::Green : FColor::Red;
	const FColor ArmColor = bIsArmed ? FColor::Green : FColor::Yellow;

	GEngine->AddOnScreenDebugMessage(101, 0.f, LinkColor,
		FString::Printf(TEXT("LINK: %s   MODE: %s   %s"),
			bLinkAlive ? TEXT("OK") : TEXT("MAT KET NOI"),
			*ModeName(CurrentMode),
			bIsArmed ? TEXT("ARMED") : TEXT("DISARMED")));

	GEngine->AddOnScreenDebugMessage(102, 0.f, FColor::Cyan,
		FString::Printf(TEXT("ALT: %.1f m    GS: %.1f m/s"), AltitudeMSL, GroundSpeed));

	GEngine->AddOnScreenDebugMessage(103, 0.f, FColor::White,
		FString::Printf(TEXT("THR %.2f  |  ROLL %+.2f  PITCH %+.2f  YAW %+.2f"),
			StickThrottle, StickRoll, StickPitch, StickYaw));

	GEngine->AddOnScreenDebugMessage(104, 0.f, ArmColor,
		TEXT("W/S throttle | A/D yaw | Mui ten roll+pitch | Enter=ARM | Backspace=DISARM | 1..6 doi mode"));
}

// ===========================================================================
//  TICK
// ===========================================================================

void ADronePawn::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);

	if (bEnableControl)
	{
		PollInput(DeltaSeconds);

		ControlSendAccum += DeltaSeconds;
		const float Interval = 1.f / FMath::Max(1.f, ControlSendHz);
		if (ControlSendAccum >= Interval)
		{
			ControlSendAccum = 0.f;
			SendControl();
		}
	}

	ApplyTelemetry(DeltaSeconds);

	if (bShowDebugOverlay)
	{
		DrawOverlay();
	}
}