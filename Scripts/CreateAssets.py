"""Run once using UnrealEditor-Cmd -run=pythonscript. Creates project-owned assets."""
import unreal

asset_tools = unreal.AssetToolsHelpers.get_asset_tools()
material_path = "/Game/Materials/M_Lab"
material = unreal.EditorAssetLibrary.load_asset(material_path) if unreal.EditorAssetLibrary.does_asset_exist(material_path) else None
if material is None:
    material = asset_tools.create_asset("M_Lab", "/Game/Materials", unreal.Material, unreal.MaterialFactoryNew())
    tint = unreal.MaterialEditingLibrary.create_material_expression(material, unreal.MaterialExpressionVectorParameter, -300, 0)
    tint.set_editor_property("parameter_name", "Tint")
    tint.set_editor_property("default_value", unreal.LinearColor(0.15, 0.4, 0.6, 1.0))
    unreal.MaterialEditingLibrary.connect_material_property(tint, "", unreal.MaterialProperty.MP_BASE_COLOR)
    roughness = unreal.MaterialEditingLibrary.create_material_expression(material, unreal.MaterialExpressionConstant, -300, 200)
    roughness.set_editor_property("r", 0.7)
    unreal.MaterialEditingLibrary.connect_material_property(roughness, "", unreal.MaterialProperty.MP_ROUGHNESS)
    unreal.MaterialEditingLibrary.recompile_material(material)
    unreal.EditorAssetLibrary.save_asset(material_path)
unreal.log("FORKLIFT_ASSETS_READY")
