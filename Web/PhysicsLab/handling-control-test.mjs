import assert from 'node:assert/strict';import {handlingControlCsv,handlingNames} from './dist/handling-control.mjs';
assert.equal(handlingControlCsv([{frame:{source:{}}}]).rows.length,0);
const h={source:'measured_local_handling_v1',state:'insert',reason:'a,"b',active:true,target_height_m:.035,vx_command_mps:.12,ground_truth_feedback:false};
const r=handlingControlCsv([{received_at_ms:1,frame:{source:{world_id:'w',run_id:'r'},sim_time_s:2,handling_control:h,fork:{actual_m:99}}}]);
assert.equal(r.columns.length,20);assert.equal(r.rows.length,1);assert(r.csv.includes('"a,""b"'));assert(!r.csv.includes('99'));assert.equal(handlingNames.insert,'자동 삽입');console.log('HANDLING_WEB_MODULE PASS: received fields only; no truth fallback; quoted CSV; empty history.');
