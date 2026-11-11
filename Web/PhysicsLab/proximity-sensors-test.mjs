import assert from 'node:assert/strict';
import {proximitySensorCsv} from './dist/proximity-sensors.mjs';
const good={status:'valid',transport_valid:true,measurement_valid:true,reading_status:'valid'};
const frame={source:{world_id:'test',run_id:'run'},sim_time_s:1,proximity_sensors:{sensors:[{id:'heel_left',kind:'range',...good,range_m:.225},{id:'presence_left_near',kind:'presence',...good,detected:false},{id:'tip_left',kind:'range',...good,measurement_valid:false,reading_status:'no_return',range_m:999},{id:'presence_left_far',kind:'presence',...good,status:'stale',transport_valid:false,detected:true}]}};
const csv=proximitySensorCsv([{received_at_ms:1,frame}]);assert.equal(csv.columns.length,17);assert.equal(csv.rows.length,4);assert.ok(csv.rows.every(r=>r.split(',').length===17));
const value=(r,k)=>csv.rows[r].split(',')[csv.columns.indexOf(k)];assert.equal(value(0,'range_m'),'0.225');assert.equal(value(1,'detected'),'false');assert.equal(value(2,'range_m'),'');assert.equal(value(3,'detected'),'');
delete frame.proximity_sensors;assert.equal(proximitySensorCsv([{frame}]).rows.length,0);
console.log('PROXIMITY_SENSOR_CSV PASS: received ranges and OFF, no-return/stale blanks, no truth fallback, 17 columns');
