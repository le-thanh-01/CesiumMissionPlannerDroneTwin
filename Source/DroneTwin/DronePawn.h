// DronePawn.h — Digital Twin DJI Neo 2  (v2: dieu khien 2 chieu)
// Unreal Engine 4.27.2 + Cesium for Unreal 1.24.0

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "Networking.h"
#include "Sockets.h"
#include "Common/UdpSocketReceiver.h"
#include "HAL/CriticalSection.h"
#include "DronePawn.generated.h"

class ACesiumGeoreference;

// ---------------------------------------------------------------------------
// Goi TELEMETRY: bridge -> Unreal (UDP 14560)
// ---------------------------------------------------------------------------
#define DRONE_STATE_MAGIC 0x314A4A44u   // 'DJI1'

#pragma pack(push, 1)
struct FDroneStatePacket
{
	uint32 Magic;
	uint32 Seq;
	double Lat;         // do (WGS84)
	double Lon;         // do (WGS84)
	double AltMSL;      // met tren muc nuoc bien
	float  Roll;        // do, duong = nghieng phai
	float  Pitch;       // do, duong = ngoc mui len
	float  Yaw;         // do, heading la ban (0 = Bac)
	float  Vx;          // m/s, NED North
	float  Vy;          // m/s, NED East
	float  Vz;          // m/s, NED Down
	float  Throttle;    // 0.0 .. 1.0
	uint8  Armed;
	uint8  Mode;
	uint16 Pad;
};
#pragma pack(pop)

static_assert(sizeof(FDroneStatePacket) == 64, "State packet phai dung 64 byte");

// ---------------------------------------------------------------------------
// Goi CONTROL: Unreal -> bridge (UDP 14561)
// ---------------------------------------------------------------------------
#define DRONE_CONTROL_MAGIC 0x324A4A44u   // 'DJI2'

// ArmRequest
#define DRONE_ARM_NONE    0
#define DRONE_ARM_ARM     1
#define DRONE_ARM_DISARM  2

// ModeRequest: 255 = khong doi. Con lai la custom_mode cua ArduCopter.
#define DRONE_MODE_NONE      255
#define DRONE_MODE_STABILIZE 0
#define DRONE_MODE_ALT_HOLD  2
#define DRONE_MODE_GUIDED    4
#define DRONE_MODE_LOITER    5
#define DRONE_MODE_RTL       6
#define DRONE_MODE_LAND      9

#pragma pack(push, 1)
struct FDroneControlPacket
{
	uint32 Magic;
	uint32 Seq;
	float  Roll;         // -1.0 .. 1.0  (duong = nghieng phai)
	float  Pitch;        // -1.0 .. 1.0  (duong = chui mui ve truoc)
	float  Yaw;          // -1.0 .. 1.0  (duong = xoay phai)
	float  Throttle;     //  0.0 .. 1.0
	uint8  ArmRequest;
	uint8  ModeRequest;
	uint16 Pad;
	uint32 Reserved;     // de danh cho gimbal / camera sau nay
};
#pragma pack(pop)

static_assert(sizeof(FDroneControlPacket) == 32, "Control packet phai dung 32 byte");


UCLASS()
class DRONETWIN_API ADronePawn : public APawn
{
	GENERATED_BODY()

public:
	ADronePawn();

	virtual void Tick(float DeltaSeconds) override;

	// ---- Components ----
	UPROPERTY(VisibleAnywhere, Category = "Drone")
		USceneComponent* RootScene;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone")
		UStaticMeshComponent* BodyMesh;

	// Bo trong neu mesh da combine. Vong lap se tu bo qua.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone")
		TArray<UStaticMeshComponent*> Propellers;

	// ---- Cesium ----
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Cesium")
		ACesiumGeoreference* Georeference = nullptr;

	// ---- Network ----
	// Cong nhan telemetry tu bridge
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Network")
		int32 ListenPort = 14560;

	// Cong gui lenh dieu khien sang bridge
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Network")
		FString BridgeHost = TEXT("127.0.0.1");

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Network")
		int32 BridgeControlPort = 14561;

	// ---- Control ----
	// Tat cai nay neu chi muon xem, khong muon lai tu Unreal.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Control")
		bool bEnableControl = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Control")
		float ControlSendHz = 25.f;

	// Toc do can roll/pitch/yaw day toi het bien (don vi/giay)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Control")
		float StickRiseRate = 3.0f;

	// Toc do can tu ve giua khi tha phim
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Control")
		float StickReturnRate = 6.0f;

	// Toc do thay doi throttle (don vi/giay). Throttle GIU nguyen khi tha phim.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Control")
		float ThrottleRate = 0.5f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Control")
		bool bUseGamepad = true;

	// Hien thong so len goc man hinh khi Play
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Control")
		bool bShowDebugOverlay = true;

	// ---- Tuning ----
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tuning")
		float YawOffsetDeg = -90.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tuning")
		float LocationInterpSpeed = 15.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tuning")
		float RotationInterpSpeed = 12.f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tuning")
		float MaxPropSpinDegPerSec = 4000.f;

	// ---- Telemetry doc duoc tu Blueprint ----
	UPROPERTY(BlueprintReadOnly, Category = "Telemetry")
		bool bIsArmed = false;

	UPROPERTY(BlueprintReadOnly, Category = "Telemetry")
		float GroundSpeed = 0.f;

	UPROPERTY(BlueprintReadOnly, Category = "Telemetry")
		float AltitudeMSL = 0.f;

	UPROPERTY(BlueprintReadOnly, Category = "Telemetry")
		bool bLinkAlive = false;

	UPROPERTY(BlueprintReadOnly, Category = "Telemetry")
		uint8 CurrentMode = 0;

	// ---- Trang thai can dieu khien, doc duoc tu Blueprint de ve HUD ----
	UPROPERTY(BlueprintReadOnly, Category = "Control")
		float StickRoll = 0.f;

	UPROPERTY(BlueprintReadOnly, Category = "Control")
		float StickPitch = 0.f;

	UPROPERTY(BlueprintReadOnly, Category = "Control")
		float StickYaw = 0.f;

	UPROPERTY(BlueprintReadOnly, Category = "Control")
		float StickThrottle = 0.f;

protected:
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

private:
	// --- nhan telemetry ---
	void StartListening();
	void StopListening();
	void OnUdpDataReceived(const FArrayReaderPtr& Data, const FIPv4Endpoint& Endpoint);
	void ApplyTelemetry(float DeltaSeconds);

	// --- gui dieu khien ---
	void StartSending();
	void StopSending();
	void PollInput(float DeltaSeconds);
	void SendControl();
	void DrawOverlay();

	FVector GeoToUnreal(double Lon, double Lat, double HeightMeters) const;

	// Socket nhan
	FSocket* RecvSocket = nullptr;
	FUdpSocketReceiver* Receiver = nullptr;

	// Socket gui
	FSocket* SendSocket = nullptr;
	TSharedPtr<FInternetAddr> BridgeAddr;

	mutable FCriticalSection StateMutex;
	FDroneStatePacket LatestPacket;
	bool bHasPacket = false;
	uint32 LastSeq = 0;
	double LastPacketTime = 0.0;

	float PropAngle = 0.f;

	// Trang thai lenh dang cho gui
	uint8 PendingArm = DRONE_ARM_NONE;
	uint8 PendingMode = DRONE_MODE_NONE;
	float PendingArmTimer = 0.f;
	float PendingModeTimer = 0.f;

	// Chong lap khi giu phim
	bool bArmKeyWasDown = false;
	bool bDisarmKeyWasDown = false;
	bool bModeKeyWasDown[6] = { false, false, false, false, false, false };

	uint32 ControlSeq = 0;
	float ControlSendAccum = 0.f;
};