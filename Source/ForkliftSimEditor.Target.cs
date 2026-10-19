using UnrealBuildTool;
public class ForkliftSimEditorTarget : TargetRules
{
    public ForkliftSimEditorTarget(TargetInfo Target) : base(Target)
    {
        Type = TargetType.Editor;
        DefaultBuildSettings = BuildSettingsVersion.V5;
        IncludeOrderVersion = EngineIncludeOrderVersion.Unreal5_6;
        ExtraModuleNames.Add("ForkliftSim");
    }
}
