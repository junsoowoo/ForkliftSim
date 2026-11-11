import assert from 'node:assert/strict';
import {lidarPoints,lidarSafetyCsv} from './dist/lidar-safety.mjs';
const bank={data_healthy:true,clearance_m:.12,state:'obstacle_hold',drive_hold:true,lift_hold:false,scanners:[{id:'front_left',fresh:true,calibrated_mount_chassis_m:[.8,-.57,-.27],start_deg:0,fov_deg:90,ranges_m:[1,null,999],return_codes:[0,1,2]}]};
assert.deepEqual(lidarPoints(bank),[{x:1.8,y:-.57,id:'front_left'}]);bank.scanners[0].fresh=false;assert.equal(lidarPoints(bank).length,0);
const frame={source:{world_id:'w',run_id:'r'},sim_time_s:1,lidar_safety:bank};let c=lidarSafetyCsv([{received_at_ms:1,frame}]);assert.equal(c.columns.length,18);assert.equal(c.rows[0].split(',')[10],'0.12');bank.data_healthy=false;c=lidarSafetyCsv([{frame}]);assert.equal(c.rows[0].split(',')[10],'');assert.equal(c.rows[0].split(',')[7],'false');console.log('LIDAR_SAFETY_UI PASS: measured point projection, stale/invalid omission, independent drive/lift holds, 18 CSV columns');
