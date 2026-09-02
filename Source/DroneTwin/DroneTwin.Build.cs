// DroneTwin.Build.cs
// Doi "DroneTwin" thanh ten project that cua ban o ca ten file lan ten class.

using UnrealBuildTool;

public class DroneTwin : ModuleRules
{
	public DroneTwin(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

		// ------------------------------------------------------------------
		// BAT BUOC khi include header cua Cesium.
		// Cesium native (CesiumGeospatial/Ellipsoid.h ...) dung std::optional,
		// von la C++17. UE 4.27 mac dinh bien dich module game bang C++14 nen
		// se bao: "The contents of <optional> are available only with C++17".
		// Hai dong duoi day mirror dung cau hinh cua module CesiumRuntime.
		// ------------------------------------------------------------------
		CppStandard = CppStandardVersion.Cpp17;
		bEnableExceptions = true;

		PublicDependencyModuleNames.AddRange(new string[]
		{
			"Core",
			"CoreUObject",
			"Engine",
			"InputCore",

			// Can cho FSocket / FUdpSocketBuilder / FUdpSocketReceiver
			"Networking",
			"Sockets",

			// Can cho ACesiumGeoreference
			"CesiumRuntime",
		});

		PrivateDependencyModuleNames.AddRange(new string[] { });
	}
}