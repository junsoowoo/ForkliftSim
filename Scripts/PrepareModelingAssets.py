from pathlib import Path
import unreal,json
root=Path(unreal.Paths.project_dir())
profile=json.loads((root/'Config/Modeling.json').read_text(encoding='utf-8'))
importer='ImportAllturnForkman.py' if profile.get('source_folder')=='AllturnForkman' else 'ImportForkliftModel.py'
for script in [importer,'InventoryWarehouse.py']:
    path=root/'Scripts'/script
    exec(compile(path.read_text(encoding='utf-8'),str(path),'exec'),{'__name__':'__main__','__file__':str(path),'MODELING_PIPELINE':True})
