// Fill out your copyright notice in the Description page of Project Settings.

using System.IO;
using UnrealBuildTool;

public class FineParticle : ModuleRules
{

    private string ModulePath
    {
        get { return ModuleDirectory; }
    }

    private string ThirdPartyPath
    {
        get { return Path.GetFullPath( Path.Combine( ModulePath, "../../ThirdParty/" ) ); }
    }

	public FineParticle(TargetInfo Target)
	{
		PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine", "InputCore" });

		PrivateDependencyModuleNames.AddRange(new string[] {  });

		// Uncomment if you are using Slate UI
		// PrivateDependencyModuleNames.AddRange(new string[] { "Slate", "SlateCore" });
		
		// Uncomment if you are using online features
		// PrivateDependencyModuleNames.Add("OnlineSubsystem");
		// if ((Target.Platform == UnrealTargetPlatform.Win32) || (Target.Platform == UnrealTargetPlatform.Win64))
		// {
		//		if (UEBuildConfiguration.bCompileSteamOSS == true)
		//		{
		//			DynamicallyLoadedModuleNames.Add("OnlineSubsystemSteam");
		//		}
		// }

        LoadCubicFineParticle(Target);
        LoadBulletPhysics(Target);
	}

    public bool LoadCubicFineParticle(TargetInfo Target)
    {
        bool isLibrarySupported = false;

        if ((Target.Platform == UnrealTargetPlatform.Win64) || (Target.Platform == UnrealTargetPlatform.Win32))
        {
            isLibrarySupported = true;

            string PlatformString = (Target.Platform == UnrealTargetPlatform.Win64) ? "" : "";
            string LibrariesPath = Path.Combine(ThirdPartyPath, "CubicFineParticle", "Libraries");

            PublicAdditionalLibraries.Add(Path.Combine(LibrariesPath, "cubic_fine_particle" + PlatformString + ".lib"));
        }

        if (Target.Platform == UnrealTargetPlatform.Mac)
        {
            isLibrarySupported = true;

            string PlatformString = "";
            string LibrariesPath = Path.Combine(ThirdPartyPath, "CubicFineParticle", "Libraries");

            PublicAdditionalLibraries.Add(Path.Combine(LibrariesPath, "libcubic_fine_particle" + PlatformString + ".a"));
        }

        if (isLibrarySupported)
        {
            // Include path
            PublicIncludePaths.Add( Path.Combine( ThirdPartyPath, "CubicFineParticle", "Includes" ) );
        }

        Definitions.Add(string.Format( "WITH_CUBIC_FINE_PARTICLE_BINDING={0}", isLibrarySupported ? 1 : 0 ) );

        return isLibrarySupported;
    }

    public bool LoadBulletPhysics(TargetInfo Target)
    {
        bool isLibrarySupported = false;

        if ((Target.Platform == UnrealTargetPlatform.Win64) || (Target.Platform == UnrealTargetPlatform.Win32))
        {
            isLibrarySupported = true;

            string PlatformString = (Target.Platform == UnrealTargetPlatform.Win64) ? "" : "";
            string LibrariesPath = Path.Combine(ThirdPartyPath, "BulletPhysics", "Libraries");

            PublicAdditionalLibraries.Add(Path.Combine(LibrariesPath, "BulletCollision" + PlatformString + ".lib"));
            PublicAdditionalLibraries.Add(Path.Combine(LibrariesPath, "BulletDynamics" + PlatformString + ".lib"));
            PublicAdditionalLibraries.Add(Path.Combine(LibrariesPath, "LinearMath" + PlatformString + ".lib"));
        }

        if (Target.Platform == UnrealTargetPlatform.Mac)
        {
            isLibrarySupported = true;

            string PlatformString = "";
            string LibrariesPath = Path.Combine(ThirdPartyPath, "BulletPhysics", "Libraries");

            PublicAdditionalLibraries.Add(Path.Combine(LibrariesPath, "libBulletCollision" + PlatformString + ".a"));
            PublicAdditionalLibraries.Add(Path.Combine(LibrariesPath, "libBulletDynamics" + PlatformString + ".a"));
            PublicAdditionalLibraries.Add(Path.Combine(LibrariesPath, "libLinearMath" + PlatformString + ".a"));
        }

        if (isLibrarySupported)
        {
            // Include path
            PublicIncludePaths.Add( Path.Combine( ThirdPartyPath, "BulletPhysics", "Includes" ) );
        }

        Definitions.Add(string.Format( "WITH_BULLET_PHYSICS_BINDING={0}", isLibrarySupported ? 1 : 0 ) );

        return isLibrarySupported;
    }

}
