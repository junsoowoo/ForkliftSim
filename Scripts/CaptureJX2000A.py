"""Capture the actual Unreal review viewport after native assets are imported."""
import unreal,time,json
from pathlib import Path
root=Path(unreal.Paths.project_dir())
levels=unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
editor=unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem)
assert levels.load_level('/Game/Maps/JX2000AReview')
editor.set_level_viewport_camera_info(unreal.Vector(-220,-300,190),unreal.Rotator(pitch=-14,yaw=48,roll=0))
world=editor.get_editor_world()
actors=unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
camera=actors.spawn_actor_from_class(unreal.SceneCapture2D,unreal.Vector(-220,-300,190),unreal.Rotator(pitch=-14,yaw=48,roll=0))
capture=camera.get_component_by_class(unreal.SceneCaptureComponent2D)
capture.set_editor_property('capture_every_frame',False)
capture.set_editor_property('capture_on_movement',False)
capture.set_editor_property('capture_source',unreal.SceneCaptureSource.SCS_FINAL_COLOR_LDR)
capture.set_editor_property('fov_angle',45)
render_target=unreal.RenderingLibrary.create_render_target2d(world,1200,1000,unreal.TextureRenderTargetFormat.RTF_RGBA8)
capture.set_editor_property('texture_target',render_target)
started=time.monotonic();requested=False;handle=None
target=root/'Docs/Images/JX2000A-Unreal.png'
if target.exists():target.unlink()
def tick(delta):
 global requested
 elapsed=time.monotonic()-started
 if not requested and elapsed>15:
  capture.capture_scene()
  unreal.RenderingLibrary.export_render_target(world,render_target,str(target.parent),target.name)
  requested=True
 if requested and target.exists() and target.stat().st_size>10000:
  unreal.log('JX_CAPTURE PASS '+str(target));unreal.unregister_slate_post_tick_callback(handle);unreal.SystemLibrary.quit_editor()
 elif elapsed>90:
  unreal.log_error('JX_CAPTURE TIMEOUT');unreal.unregister_slate_post_tick_callback(handle);unreal.SystemLibrary.quit_editor()
handle=unreal.register_slate_post_tick_callback(tick)
