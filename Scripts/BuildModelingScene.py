from pathlib import Path
import unreal
root=Path(unreal.Paths.project_dir())
exec(compile((root/'Scripts/ImportForkliftModel.py').read_text(encoding='utf-8'),str(root/'Scripts/ImportForkliftModel.py'),'exec'))
exec(compile((root/'Scripts/CreateWarehouseLab.py').read_text(encoding='utf-8'),str(root/'Scripts/CreateWarehouseLab.py'),'exec'))
