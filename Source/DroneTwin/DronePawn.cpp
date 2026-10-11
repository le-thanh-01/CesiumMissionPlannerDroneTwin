// DronePawn.cpp  (v2: dieu khien 2 chieu)

#include "DronePawn.h"
#include "CesiumGeoreference.h"
#include "Kismet/GameplayStatics.h"
#include "SocketSubsystem.h"
#include "GameFramework/PlayerController.h"
#include "Engine/Engine.h"

// ---------------------------------------------------------------------------
//  Camera TPV / FPV
//
//  SpringArm trong class nay co bInheritPitch/Roll = false. Khi bInheritYaw = false
//  (FPV), UE coi RelativeRotation cua arm la goc THE GIOI, khong phai goc so voi
//  drone. Vi vay muon camera nhin ra truoc mui drone thi phai gan Yaw = yaw hien
//  tai cua actor, khong the dung ZeroRotator.
//  Khi bInheritYaw = true (TPV), Yaw = 0 nghia la camera nam sau duoi drone.
// ---------------------------------------------------------------------------
namespace DroneCam
{
	// SpringArm KHONG tu scale theo actor/mesh: TargetArmLength va SocketOffset luon
	// tinh bang don vi the gioi (cm). Mesh to gap N lan thi chi can dat CamScale = N.

	static const float TPVArmLength = 800.f  ;
	static const float TPVPitch = -15.f;
	static const FVector TPVSocketOffset = FVector(0.f, 0.f, 150.f)  ;
	static const FVector FPVSocketOffset = FVector(15.f, 0.f, 5.f)  ;

	
	static void ApplyTPV(USpringArmComponent* Arm, float ArmLength, const FVector& SocketOffset, float Pitch)
	{
		Arm->TargetArmLength = ArmLength;
		Arm->SocketOffset = SocketOffset;
		Arm->bInheritYaw = true;
		Arm->SetRelativeRotation(FRotator(Pitch, 0.f, 0.f));
	}

	// FPV: dat camera nhin thang ra truoc mui drone (yaw the gioi = yaw actor, pitch = 0).
	static void RecenterFPV(USpringArmComponent* Arm, const AActor* Owner)
	{
		Arm->SetRelativeRotation(FRotator(0.f, Owner->GetActorRotation().Yaw, 0.f));
	}

	static void ApplyFPV(USpringArmComponent* Arm, const AActor* Owner, const FVector& SocketOffset)
	{
		Arm->TargetArmLength = 0.f;
		Arm->SocketOffset = SocketOffset;
		Arm->bInheritYaw = false;
		RecenterFPV(Arm, Owner);
	}
	
}

namespace DroneInput
{
	// Dao chieu stick roll/pitch (ban phim + tay cam). Dat 1.f de tra lai huong cu.
	//static const float RollSign = -1.f;
	static const float PitchSign = -1.f;
	// Toc do xoay camera FPV khi giu nut Y/A/X/B tren tay cam (do/giay)
	static const float GimbalButtonDegPerSec = 90.f;
}

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
	// Đổi từ TEXT("SpringArm") thành TEXT("SpringArm_CPP")
	SpringArm = CreateDefaultSubobject<USpringArmComponent>(TEXT("SpringArm_CPP"));
	SpringArm->SetupAttachment(RootScene);
	CamTPVArmLength = DroneCam::TPVArmLength;
	CamTPVSocketOffset = DroneCam::TPVSocketOffset;
	CamTPVPitch = DroneCam::TPVPitch;
	CamFPVSocketOffset = DroneCam::FPVSocketOffset;
	// Đổi từ TEXT("Camera") thành TEXT("Camera_CPP")
	Camera = CreateDefaultSubobject<UCameraComponent>(TEXT("Camera_CPP"));
	Camera->SetupAttachment(SpringArm);
	// ... (các đoạn code trước đó giữ nguyên)
	SpringArm->TargetArmLength = DroneCam::TPVArmLength; // Độ dài TPV mặc định
	SpringArm->SocketOffset = FVector(0.f, 0.f, 150.f);
	SpringArm->bDoCollisionTest = false;
	SpringArm->bInheritPitch = false;
	SpringArm->bInheritRoll = false;
	SpringArm->bInheritYaw = true;
	SpringArm->bEnableCameraLag = false;
	SpringArm->SetRelativeRotation(FRotator(-15.f, 0.f, 0.f));

	AutoPossessPlayer = EAutoReceiveInput::Disabled; // Không tự động nhận input, để người chơi tự chọn
}

void ADronePawn::BeginPlay()
{
	Super::BeginPlay();
	if (Models.Num() > 0)
	{
		SetModel(FMath::Clamp(DefaultModelIndex, 0, Models.Num() - 1));
	}
	// Gom cac component ten bat dau bang "Motor", sap theo ten: Motor1.. Motor4 = motor 1..4 cua ArduPilot
	if (Propellers.Num() == 0)
	{
		TArray<UStaticMeshComponent*> Meshes;
		GetComponents<UStaticMeshComponent>(Meshes);
		Meshes.RemoveAll([](UStaticMeshComponent* C) { return !C || !C->GetName().StartsWith(TEXT("Motor")); });
		Meshes.Sort([](const UStaticMeshComponent& A, const UStaticMeshComponent& B) { return A.GetName() < B.GetName(); });
		Propellers = Meshes;
	}

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
	if (!SpringArm)
	{
		return;
	}

	// Dang possess drone -> dung controller cua no.
	// Khong possess (vi du dang lam pilot) -> van doc ban phim cua nguoi choi 0
	// de lai drone "tu xa". Luc nay camera la cua pilot nen bo qua V/F/gimbal.
	APlayerController* PC = Cast<APlayerController>(GetController());
	const bool bRemote = (PC == nullptr);
	if (bRemote)
	{
		PC = UGameplayStatics::GetPlayerController(this, 0);
	}

	if (!PC)
	{
		// Khong co nguoi choi nao: can roll/pitch/yaw ve giua nhu lo xo, throttle giu nguyen,
		// lenh ARM/MODE dang cho cung het han binh thuong.
		StickRoll = FMath::FInterpConstantTo(StickRoll, 0.f, DeltaSeconds, StickReturnRate);
		StickPitch = FMath::FInterpConstantTo(StickPitch, 0.f, DeltaSeconds, StickReturnRate);
		StickYaw = FMath::FInterpConstantTo(StickYaw, 0.f, DeltaSeconds, StickReturnRate);

		PendingArmTimer = FMath::Max(0.f, PendingArmTimer - DeltaSeconds);
		if (PendingArmTimer <= 0.f) PendingArm = DRONE_ARM_NONE;

		PendingModeTimer = FMath::Max(0.f, PendingModeTimer - DeltaSeconds);
		if (PendingModeTimer <= 0.f) PendingMode = DRONE_MODE_NONE;
		return;
	}
	if (PC->WasInputKeyJustPressed(EKeys::M))
	{
		NextModel();
	}
	// --- 1. Phim V: doi TPV <-> FPV ---
	if (!bRemote && (PC->WasInputKeyJustPressed(EKeys::V) || PC->WasInputKeyJustPressed(EKeys::Gamepad_RightShoulder)))
	{
		bIsFPV = !bIsFPV;
		if (bIsFPV) DroneCam::ApplyFPV(SpringArm, this, CamFPVSocketOffset);
		else        DroneCam::ApplyTPV(SpringArm, CamTPVArmLength, CamTPVSocketOffset, CamTPVPitch);
	}

	// --- 2. Phim F: dua camera FPV ve phia truoc drone (chi co tac dung o FPV) ---
	if (!bRemote && bIsFPV && (PC->WasInputKeyJustPressed(EKeys::F) || PC->WasInputKeyJustPressed(EKeys::Gamepad_RightThumbstick)))
	{
		DroneCam::RecenterFPV(SpringArm, this);
	}

	// --- 3. Gimbal camera FPV (luc dang possess drone): chuot + nut Y/A/X/B tren tay cam ---
	if (!bRemote && bIsFPV)
	{
		float MouseX = 0.f;
		float MouseY = 0.f;
		PC->GetInputMouseDelta(MouseX, MouseY);

		float YawDelta = MouseX * MouseSensitivity;
		float PitchDelta = MouseY * MouseSensitivity;

		// Giu Y = nhin len, A = nhin xuong, X = quay trai, B = quay phai
		const float ButtonStep = DroneInput::GimbalButtonDegPerSec * DeltaSeconds;
		if (PC->IsInputKeyDown(EKeys::Gamepad_FaceButton_Top))    PitchDelta += ButtonStep;
		if (PC->IsInputKeyDown(EKeys::Gamepad_FaceButton_Bottom)) PitchDelta -= ButtonStep;
		if (PC->IsInputKeyDown(EKeys::Gamepad_FaceButton_Left))   YawDelta -= ButtonStep;
		if (PC->IsInputKeyDown(EKeys::Gamepad_FaceButton_Right))  YawDelta += ButtonStep;

		if (FMath::Abs(YawDelta) > KINDA_SMALL_NUMBER || FMath::Abs(PitchDelta) > KINDA_SMALL_NUMBER)
		{
			FRotator CurrentArmRot = SpringArm->GetRelativeRotation();

			CurrentArmRot.Yaw += YawDelta;
			CurrentArmRot.Pitch = FMath::Clamp(CurrentArmRot.Pitch + PitchDelta, -80.f, 80.f);

			SpringArm->SetRelativeRotation(CurrentArmRot);
		}
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

	// --- Dao chieu roll ---
	TargetPitch *= DroneInput::PitchSign;

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

	const FVector TargetLocation = GeoToUnreal(Packet.Lon, Packet.Lat, Packet.AltMSL - 30);
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
	bIsArmed = (Packet.Armed != 0);
	AltitudeMSL = static_cast<float>(Packet.AltMSL);
	GroundSpeed = FMath::Sqrt(Packet.Vx * Packet.Vx + Packet.Vy * Packet.Vy);
	CurrentMode = Packet.Mode;
	// Moi dong co quay theo PWM rieng cua no
	for (int32 i = 0; i < Propellers.Num() && i < 4; ++i)
	{
		if (!Propellers[i]) continue;
		const float Pwm = static_cast<float>(Packet.MotorPwm[i]);
		const float Norm = (Pwm < 900.f) ? 0.f
			: FMath::Clamp((Pwm - MotorPwmMin) / FMath::Max(1.f, MotorPwmMax - MotorPwmMin), 0.f, 1.f);

		// Quad X ArduCopter: motor 1,2 quay CCW, motor 3,4 quay CW.
		// Unreal: Yaw duong = cung chieu kim dong ho khi nhin tu tren xuong.
		const float Dir = (i < 2) ? -1.f : 1.f;
		Propellers[i]->AddLocalRotation(FRotator(0.f, Dir * Norm * MaxPropSpinDegPerSec * DeltaSeconds, 0.f));
	}
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

	GEngine->AddOnScreenDebugMessage(999, 0.f, FColor::Black, "Shift + / or Left Stick Click to help");

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

//Hàm thay đổi bộ khung mới cho drone
void ADronePawn::SetModel(int32 Index)
{
	if (!Models.IsValidIndex(Index) || !BodyMesh)
	{
		if (GEngine)
		{
			GEngine->AddOnScreenDebugMessage(110, 2.f, FColor::Orange,
				TEXT("Loi MODEL! Vui long kiem tra lai"));
		}
		return;
	}
	if (bIsArmed)
	{
		if (GEngine)
		{
			GEngine->AddOnScreenDebugMessage(110, 2.f, FColor::Orange,
				TEXT("Can DISARM truoc khi doi MODEL!"));
		}
		return;
	}
	const FDroneModelPreset& M = Models[Index];
	CurrentModelIndex = Index;

	// 1. Than drone
	if (M.BodyMeshAsset)
	{
		BodyMesh->SetStaticMesh(M.BodyMeshAsset);
	}
	BodyMesh->SetRelativeRotation(M.BodyRotation);
	BodyMesh->SetRelativeScale3D(M.BodyScale);
	BodyMesh->SetRelativeLocation(M.BodyLocation);
	// 2. Xoa motor cua mo hinh truoc, tao motor moi theo thu tu 1..4
	for (UStaticMeshComponent* Old : SpawnedMotors)
	{
		if (Old)
		{
			Old->DestroyComponent();
		}
	}
	SpawnedMotors.Reset();
	Propellers.Reset();

	for (int32 i = 0; i < M.Motors.Num(); ++i)
	{
		const FDroneMotorSlot& Slot = M.Motors[i];
		if (!Slot.Mesh)
		{
			Propellers.Add(nullptr);   // giu dung chi so motor
			continue;
		}

		// Ten dang "Motor1_0": van bat dau bang "Motor" + so, khong trung ten voi component cu
		const FName CompName = MakeUniqueObjectName(this, UStaticMeshComponent::StaticClass(),
			*FString::Printf(TEXT("Motor%d"), i + 1));

		// Neu mesh than co socket ten Motor1..Motor4 thi gan vao socket do
		const FName SocketName(*FString::Printf(TEXT("Motor%d"), i + 1));
		const bool bHasSocket = BodyMesh->DoesSocketExist(SocketName);

		UStaticMeshComponent* C = NewObject<UStaticMeshComponent>(this, CompName);
		C->SetStaticMesh(Slot.Mesh);
		C->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		C->SetupAttachment(BodyMesh, bHasSocket ? SocketName : NAME_None);
		C->RegisterComponent();
		C->SetRelativeLocationAndRotation(Slot.Location, Slot.Rotation);   // co socket: day la do lech so voi socket
		C->SetRelativeScale3D(Slot.Scale);

		SpawnedMotors.Add(C);
		Propellers.Add(C);
	}

	// 3. Camera rieng cua mo hinh
	CamTPVArmLength = M.TPVArmLength;
	CamTPVSocketOffset = M.TPVSocketOffset;
	CamTPVPitch = M.TPVPitch;
	CamFPVSocketOffset = M.FPVSocketOffset;

	if (SpringArm)
	{
		if (bIsFPV) DroneCam::ApplyFPV(SpringArm, this, CamFPVSocketOffset);
		else        DroneCam::ApplyTPV(SpringArm, CamTPVArmLength, CamTPVSocketOffset, CamTPVPitch);
	}
}

void ADronePawn::NextModel()
{
	if (bIsArmed)
	{
		if (GEngine)
		{
			GEngine->AddOnScreenDebugMessage(110, 2.f, FColor::Orange,
				TEXT("Can DISARM truoc khi doi MODEL!"));
		}
		return;
	}
	if (Models.Num() > 1)
	{
		SetModel((CurrentModelIndex + 1) % Models.Num());
	}
}

void ADronePawn::ReapplyModel()
{
	SetModel(CurrentModelIndex);
}