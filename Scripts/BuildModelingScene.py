from pathlib import Path
import unreal,json
root=Path(unreal.Paths.project_dir())
MODELING_PIPELINE=True
profile=json.loads((root/'Config/Modeling.json').read_text(encoding='utf-8'))
importer='ImportAllturnForkman.py' if profile.get('source_folder')=='AllturnForkman' else 'ImportForkliftModel.py'
exec(compile((root/'Scripts'/importer).read_text(encoding='utf-8'),str(root/'Scripts'/importer),'exec'))
exec(compile((root/'Scripts/CreateWarehouseLab.py').read_text(encoding='utf-8'),str(root/'Scripts/CreateWarehouseLab.py'),'exec'))
