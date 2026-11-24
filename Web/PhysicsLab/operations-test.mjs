// Explicit synthetic display fixture. Native physical results are separate.
import assert from 'node:assert/strict';
import {fleetCsv,fleetPanel} from './dist/fleet.mjs';
const vehicles=Array.from({length:6},(_,i)=>({vehicle_id:`AGV-0${i+1}`,sim_time_s:1,vehicle:{speed_mps:.1},fork:{height_m:0},drive_estimator:{localized:false},campus_job:{kind:'charge',charger_id:'P01',state:'to_charger',path_m:[]}}));
const energy={enabled:true,completed_charges:0,vehicles:vehicles.map(v=>({vehicle_id:v.vehicle_id,soc:.24,stored_wh:5356.8,consumed_wh:7,charged_wh:2,power_w:123,charging:false,mode:'to_charger'})),chargers:[{id:'P01',owner:'AGV-01',position_m:[-32,5]}]};
const frame={source:{world_id:'display_fixture',sequence:1},fleet:{vehicles,slots:[],traffic_claims:[],waiting_jobs:1,traffic_waits:[{vehicle_id:'AGV-02',blocked_by:'AGV-01'}],deadlock_vehicles:[],task_board:{traffic_policy:'section_windows',energy}}};
const rows=fleetCsv([{received_at_ms:1,frame}]).split('\n').map(r=>r.split(','));
assert.equal(rows.length,7);assert.equal(rows[0].length,29);assert.ok(rows[0].includes('"battery_soc"'));assert.equal(rows[1][23],'"0.24"');assert.equal(rows[1][25],'"7"');
const html=fleetPanel(frame,'AGV-01',false);assert.ok(html.includes('24.00%'));assert.ok(html.includes('구간 예약'));assert.ok(html.includes('충전 작업 P01'));assert.ok(html.includes('AGV-02 → AGV-01'));assert.ok(!html.includes('<circle'));
delete frame.fleet.task_board.energy;assert.equal(fleetCsv([{received_at_ms:1,frame}]).split('\n')[1].split(',')[23],'""');assert.ok(!fleetPanel(frame,'AGV-01').includes('배터리 0.00%'));
console.log('OPERATIONS_DISPLAY PASS 12 assertions; synthetic fixture, no generated live physics');
