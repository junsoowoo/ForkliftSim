using UnrealBuildTool;
public class ForkliftSim : ModuleRules
{
    public ForkliftSim(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
        // Test translation units have independent anonymous-namespace helpers.
        // Keep clean/tracked checkouts independent of adaptive unity grouping.
        bUseUnity = false;
        PublicDependencyModuleNames.AddRange(new string[] {
            "Core", "CoreUObject", "Engine", "InputCore", "PhysicsCore", "Chaos", "Json", "MeshDescription", "StaticMeshDescription"
        });
        PrivateDependencyModuleNames.AddRange(new string[] { "HTTP", "RenderCore", "RHI" });
    }
}
