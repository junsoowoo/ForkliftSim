from pathlib import Path
import unreal
root=Path(unreal.Paths.project_dir())
for script in ['ImportForkliftModel.py','InventoryWarehouse.py']:
    path=root/'Scripts'/script
    exec(compile(path.read_text(encoding='utf-8'),str(path),'exec'),{'__name__':'__main__','__file__':str(path)})
