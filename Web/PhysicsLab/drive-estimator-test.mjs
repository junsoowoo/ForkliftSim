import assert from 'node:assert/strict';
import {driveEstimatorCsv} from './dist/drive-estimator.mjs';
const d={state:'ready',data_healthy:true,calibrated:true,localized:true,vx_mps:.42,vy_mps:.12,odom_x_m:2,map_x_m:1.7,torque_commands_nm:[1,2,3,4]},frame={source:{world_id:'w',run_id:'r'},sim_time_s:2,drive_estimator:d,vehicle:{speed_mps:999,position_m:[999,999,999]}};
let c=driveEstimatorCsv([{frame}]);assert.equal(c.columns.length,26);assert.equal(c.rows[0].split(',').length,26);assert.equal(c.rows[0].split(',')[9],'0.42');assert.equal(c.rows[0].split(',')[16],'1.7');
d.localized=false;c=driveEstimatorCsv([{frame}]);assert.equal(c.rows[0].split(',')[16],'');d.state='fault_latched';d.data_healthy=false;c=driveEstimatorCsv([{frame}]);assert.equal(c.rows[0].split(',')[9],'');assert.equal(c.rows[0].split(',')[13],'');assert(!c.csv.includes('999'));
console.log('DRIVE_ESTIMATOR_CSV PASS: 26 columns, received-only estimates, fault/unlocalized blanks, no truth fallback');
