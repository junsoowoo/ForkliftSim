using UnrealBuildTool;
public class ForkliftSimTarget : TargetRules
{
    public ForkliftSimTarget(TargetInfo Target) : base(Target)
    {
        Type = TargetType.Game;
        DefaultBuildSettings = BuildSettingsVersion.V5;
        IncludeOrderVersion = EngineIncludeOrderVersion.Unreal5_6;
        ExtraModuleNames.Add("ForkliftSim");
    }
}
