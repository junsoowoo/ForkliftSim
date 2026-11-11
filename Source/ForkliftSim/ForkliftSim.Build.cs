using UnrealBuildTool;
public class ForkliftSim : ModuleRules
{
    public ForkliftSim(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
        PublicDependencyModuleNames.AddRange(new string[] {
            "Core", "CoreUObject", "Engine", "InputCore", "PhysicsCore", "Chaos", "Json", "MeshDescription", "StaticMeshDescription"
        });
        PrivateDependencyModuleNames.AddRange(new string[] { "HTTP", "RenderCore", "RHI" });
    }
}
