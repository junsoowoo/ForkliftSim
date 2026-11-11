"""Collect functional evidence and report performance qualification separately."""
import argparse,hashlib,json,re
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
def read(p):return json.loads(Path(p).read_text(encoding='utf-8-sig'))
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def artifact(p):
 p=Path(p);return {'path':str(p.relative_to(ROOT)) if p.is_relative_to(ROOT) else str(p),'sha256':sha(p)}
def main():
 p=argparse.ArgumentParser();p.add_argument('suite');p.add_argument('realtime');p.add_argument('cameras');p.add_argument('default');a=p.parse_args();suite=Path(a.suite);rt=Path(a.realtime);camera=Path(a.cameras);default=Path(a.default);validation=read(suite/'mission.json');dll=sha(ROOT/'Binaries/Win64/UnrealEditor-ForkliftSim.dll');assert validation['pass'] and validation['dll_sha256']==dll
 manifests=[]
 for name in ['mission','handling-regression','drive-physics-regression']:
  text=(suite/(name+'.txt')).read_text(encoding='utf-8-sig');folder=Path(re.search(r'Mecanum report: ([^\r\n]+)',text)[1]);m=read(folder/'manifest.json');assert m['dll_sha256'].lower()==dll and sha(ROOT/'Scripts/Test-Mecanum.ps1')==m['test_script_sha256'].lower()
  for f in m['sources']+m['configs']:assert sha(f['Path'])==f['Hash'].lower(),f['Path']
  manifests.append({**artifact(folder/'manifest.json'),'conditions':len(read(folder/'results.json')),'rendered':m['rendered'],'current_dll_sources_configs_and_test_script_match':True})
 unit=read(ROOT/'Saved/Validation/Mission-QualifiedUnit/index.json');assert unit['succeeded']==89 and unit['failed']==unit['notRun']==0
 um=read(ROOT/'Saved/Validation/Mission-QualifiedUnit/manifest.json');assert um['dll_sha256'].lower()==dll
 for f in um['sources']+um['configs']:assert sha(f['Path'])==f['Hash'].lower()
 rm=read(rt/'manifest.json');assert rm['dll_sha256'].lower()==dll
 for f in rm['sources']+rm['configs']:assert sha(f['Path'])==f['Hash'].lower()
 result=read(rt/'result.json');assert result['pass'] and not result['benchmark'] and result['web_frames']>20
 dm=read(default/'manifest.json');dr=read(default/'result.json');assert dm['dll_sha256'].lower()==dll and dm['full_mission'] and dr['pass'] and dr['full_mission'] and not dr['benchmark']
 for f in dm['sources']+dm['configs']:assert sha(f['Path'])==f['Hash'].lower()
 df=read(default/'web-frames.json');last=df[-1]['frame']['mission'];assert last['state']=='complete' and last['pickup_receipt'] and last['dropoff_receipt']
 web=read(ROOT/'Docs/References/MissionWeb-20261107.json');assert web['pass'] and len(web['cases'])==8
 assert web['stream_sha256']==sha(ROOT/'Saved/Validation/Mission-Live.jsonl') and web['csv_sha256']==sha(ROOT/'Saved/Validation/Mission-WebExport.csv')
 assert 'CAMERA_SUITE PASS' in (ROOT/'Saved/Validation/Performance/Mission-CameraConsole.txt').read_text(encoding='utf-8-sig')
 assert 'PASS 32 checks' in (ROOT/'Saved/Validation/Performance/Mission-Bridge.txt').read_text(encoding='utf-8-sig')
 artifacts=[suite/'mission.json',ROOT/'Saved/Validation/Mission-QualifiedUnit/index.json',ROOT/'Saved/Validation/Mission-QualifiedUnit/manifest.json',rt/'manifest.json',rt/'performance.json',rt/'result.json',rt/'web-frames.json',default/'manifest.json',default/'performance.json',default/'result.json',default/'web-frames.json',default/'runtime.log',camera/'cameras.json',ROOT/'Docs/References/MissionWeb-20261107.json',ROOT/'Docs/MissionNavigation.png',ROOT/'Saved/Validation/MissionPlotInputs.json',ROOT/'Saved/Validation/Performance/Mission-WebModules.txt',ROOT/'Saved/Validation/Performance/Mission-Bridge.txt']
 files=['Start.bat','Scripts/Start.ps1','Scripts/Run.ps1','Scripts/Test-Mission.ps1','Scripts/Test-MissionRealtime.ps1','Scripts/Test-Cameras.ps1','Scripts/validate_mission.py','Scripts/plot_mission.py','Scripts/write_mission_evidence.py','Web/PhysicsLab/verify-mission-live.mjs','Web/PhysicsLab/mission-test.mjs','Web/PhysicsLab/bridge-test.mjs','Web/PhysicsLab/dist/mission.mjs','Web/PhysicsLab/dist/live.mjs','Web/PhysicsLab/dist/live.html','Web/PhysicsLab/dist/data.mjs','Web/PhysicsLab/bridge.mjs','Web/PhysicsLab/build.mjs','Docs/MissionNavigation.md','Docs/AutomationBatches.md','README.md']
 out={'date':'2026-11-07','pass':True,'realtime_target_passed_both_samples':bool(result['meets_realtime_target'] and dr['meets_realtime_target']),'dll_sha256':dll,'unit':{'succeeded':89,'failed':0},'native_conditions':12,'native_manifests':manifests,'web':web,'realtime_sample':result,'default_warehouse_full_mission':dr,'artifacts':[artifact(x) for x in artifacts],'implementation_and_launchers':[artifact(ROOT/x) for x in files],'scope':validation['scope'],'excluded_candidates':'Builds 1–13 and earlier full-suite/Trial candidates include development failures or older sources and are not counted. Only the current DLL/source/config manifests above qualify. pass means functional checks; performance qualification is reported separately. The 20-second sample and default-warehouse full-cycle performance are separate measurements, neither certifies Editor PIE or all conditions. Camera regression uses actual GPU targets; web checks include received data/module export, not DOM-click downloads.'}
 (ROOT/'Docs/References/MissionEvidence-20261107.json').write_text(json.dumps(out,ensure_ascii=False,indent=2)+'\n',encoding='utf-8');print(json.dumps({'pass':True,'unit':89,'native_conditions':12,'web_frames':web['actual_frames'],'realtime_ratio':result['actual_realtime_ratio']}))
if __name__=='__main__':main()
