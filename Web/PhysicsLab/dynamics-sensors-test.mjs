import assert from 'node:assert/strict';
import {dynamicsSensorCsv} from './dist/dynamics-sensors.mjs';
const frame={source:{world_id:'test',run_id:'run'},sim_time_s:4,hydraulics:{pressure_pa:123456},dynamics_sensors:{sensors:[{id:'lift_pressure',status:'valid',valid:true,gauge_pressure_pa:321000,sequence:9},{id:'imu',status:'stale',valid:false,specific_force_x_mps2:999},{id:'fork_load_left',status:'waiting',valid:false}]}};
const csv=dynamicsSensorCsv([{received_at_ms:1,frame}]);
assert.equal(csv.columns.length,21);assert.equal(csv.rows.length,3);assert.ok(csv.rows.every(r=>r.split(',').length===21));
const p=csv.rows[0].split(',');assert.equal(p[csv.columns.indexOf('gauge_pressure_pa')],'321000');
for(const r of csv.rows.slice(1))for(const k of ['specific_force_x_mps2','gauge_pressure_pa','fork_axis_force_n'])assert.equal(r.split(',')[csv.columns.indexOf(k)],'');
delete frame.dynamics_sensors;assert.equal(dynamicsSensorCsv([{frame}]).rows.length,0);
console.log('DYNAMICS_SENSOR_CSV PASS: received-only measurements, stale blanks, no truth fallback, 21 columns');
