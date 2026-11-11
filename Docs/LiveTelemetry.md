# 0.12.0 — Unreal 실시간 웹 계측

0.16에서는 `drive_type=mecanum`, `wheels[].mecanum.rollers[]`의 실제 롤러별 위치·축·질량·각속도·수직반력·평균 베어링 토크·관절 오차를 추가했다. `energy_audit.roller_bearing_loss_j`는 물리 스텝의 각충격량과 상대 회전으로 적분한다. 메카넘은 `steerable=false`, `estimated_patch_valid=false`, `tread.enabled=false`이며 이전 타이어의 눌림·면압을 재사용하지 않는다. 전체 에너지 폐합과 정격 하중은 미검증이다. [메카넘 범위](Mecanum.md)를 참고한다.

3+4에서는 `energy_audit`와 `fork.tines[].elastic_j`를 추가했다. 기준 프레임 이후 저장 변화, 실제 구동/조향 관절 일, 유압 DC/열, 타이어 모델 감쇠·구름저항과 signed 잔차를 표시한다. 감쇠 속도는 세계 좌표의 연결점 상대 이동을 현재 관절축으로 투영한다. 서로 다른 프레임의 로컬 좌표 차이를 쓰던 초기 관측치는 현재 손실값으로 재사용하지 않는다. `full_closure=false`이며 접촉·제동·베어링·구조 손실과 수치 오차는 아직 잔차에 함께 남는다. 외부 가압 벤치의 외력이 하는 일도 이 예산 밖이므로 파손 시험은 에너지 합산 판정에서 제외한다. [계측 경계와 검증](NumericalCalibration.md)을 참고한다.

0.15.0의 `Start.bat -CoupledTread`는 실제 연결 링의 `wheels[].tread.coupled`, `aggregate_shear_spring=false`, `links`, `max_shear_m`, `max_rotation_rad`, `shear_stored_j`, `rotation_stored_j`, `link_stored_j`와 조각별 `shear_m`, `rotation_rad`를 전송한다. 연결 링은 `estimated_patch_valid=false`이며 기존 기하 근사 패치·면압을 웹에서 숨긴다. `resolution_qualified=false`도 유지한다. [새 모델의 시험·미통과 조건](CoupledTread.md)을 참고한다.

0.14.1의 분할 타이어는 `wheels[].tread`에 조각 수·실제 변형/반력/위치와 `resolution_qualified=false`, `phase_rad`를 전송한다. `resolution_qualified`는 모델 검증 상태이며 센서나 실시간 오차 계산값이 아니다. 웹은 해상도 수렴 미확인·실측 미보정을 표시하고, 32조각에는 한쪽 턱 스트로크 기준 미통과 검증 기록을 함께 표시한다. `playback_rate`는 시간 배속 **설정값**이다. 실제 속도 보장을 뜻하지 않으므로 누적 물리 시간·월드 시간과 PC 부하를 함께 확인한다. [해상도 검증 결과](TreadResolution.md)를 참고한다.

## 실행

`Start.bat` → Unreal Editor와 로컬 웹 대시보드가 함께 열림 → Editor에서 Play → Unreal에서 W/S·A/D·E/Q 조작.

- W로 팔레트 구멍에 포크 삽입 후 E로 상승, Q로 하강한다. 웹의 포크 반력, 바퀴 반력, 마찰력과 유압이 실제 측정 상태를 따라 바뀐다.
- 1/2 타이어 마찰, 3/4 노면 마찰을 바꾸면 웹의 설정값과 바퀴별 실제 접촉 μ가 갱신된다.
- R 리셋은 실행 ID를 바꾸고 웹 그래프를 초기화한다. Play를 종료하면 종료 상태, 갱신이 1.5초 이상 멈추면 일시정지/수신 중단을 표시한다.
- `Start.bat -Website`는 웹만 연다. `Start.bat -NoWebsite`는 웹 없이 Unreal만 연다. `-WebPort 4174`처럼 포트를 변경하면 송신 주소와 브라우저 주소가 같이 변경된다.
- 기본 주소: `http://127.0.0.1:4174`. Node.js는 기존 설치 또는 이 PC의 Codex 제공 런타임을 자동 탐색한다. 설치 위치가 달라 Node를 찾을 수 없으면 오류를 표시한다.

직접 `.uproject`를 여는 대신 `Start.bat`를 사용한다. 수동 명령에는 `-WebTelemetry=http://127.0.0.1:4174/api/telemetry`가 필요하다. 이전 정지식 Lift/평면 Drive 모드는 이번 주행 차체 계측 대상이 아니다.

## 데이터 경로

1. `ContactLoadSampler`가 물리 스레드의 접촉 충격량을 누적한다.
2. `AStabilityLab`가 출력 구간 시간으로 평균력으로 바꾸고 차량·포크·화물·유압의 실제 상태를 모은다.
3. `WebTelemetry.cpp`가 게임 스레드에서 최대 20 Hz(벽시계 기준), 한 번에 한 개의 비동기 HTTP 요청으로 루프백 서버에 전송한다. 서버 미실행 시 재시도하며 힘·위치·설정을 쓰거나 물리 진행을 대기시키지 않는다.
4. `Web/PhysicsLab/bridge.mjs`가 최신 상태와 제한된 기록을 보관하고 SSE `/events`로 브라우저에 전달한다.
5. `live.mjs`가 수신한 값의 단위 변환·표시·그래프를 담당한다. 바퀴 하중이나 유압을 브라우저에서 다시 물리 계산하지 않는다.

측정값의 원본 SI 단위는 N, m, s, Pa, Nm, W, J다. `measurement_interval_s`는 접촉 평균의 실제 시간 구간이며 20 Hz 전송률이나 내부 물리 스텝과 구분한다. 접촉 출력이 아직 없으면 `contact_sample_valid=false`로 하중을 표시하지 않는다. 기본 바퀴 하중은 원시 평균력이고 80 ms 필터 표시를 별도로 선택할 수 있다.

## 표시 항목

- 차체: 속도, 횡속도, 가속도, 롤/피치, 요 각속도, 위치, 질량, 바퀴 수직반력 합.
- 입력: 실제 브레이크 입력(0~1), 가속·조향 입력, 바퀴별 브레이크 토크 한도. 웹의 ‘스페이스 브레이크 입력’과 Unreal HUD의 ‘SPACE brake ON/OFF’로 입력 도달 여부를 구분한다.
- 바퀴별: 원시/필터 하중, 접선 마찰력 벡터 및 크기, 접촉 μ, 접촉 이벤트 수, 눌림, 회전 속도, 실제 조향각, 종/횡슬립, 구동/제동/구름/베어링 토크, 손실 전력.
- 포크별: 실제 수직·법선·마찰 반력, 삽입 겹침, 접촉 압력중심, 실제 높이와 목표 높이.
- 화물: 질량, 위치·상대 이동, 접촉 법선/마찰력, 미끄럼 속도, 기울기, 지면 반력, 낙하·전도·지지 상실 판정.
- 유압: 실제 압력·실린더 힘·밸브·펌프/복귀/누설/릴리프 유량·전력·에너지·유지 상태.

바퀴 마찰은 접선 충격량 **벡터 합 / 출력 시간**, 포크·화물 마찰 크기는 접촉점별 접선 충격량 **크기 합 / 출력 시간**이다. 상쇄되는 접촉점 반력 때문에 둘을 같은 통계로 해석하지 않는다. μN 대비 사용량만 측정 N과 실제/설정 μ로 산출한 참고값이다. 물리 힘을 이 한계에 맞춰 수정하지 않는다.

상부 도식은 실제 차체 기준 위치(메시 크기 스케일을 제거한 좌표)·조향·하중을 표시한다. Unreal 화면 영상 스트리밍은 아니다. 웹의 질량에는 지면에 놓인 화물도 포함하므로 장비가 실제 받치는 중량과 구분한다.

## 로컬 연결과 상태

루프백 `127.0.0.1`에만 바인딩한다. 클라우드/Sites 게시·터널·외부 데이터 전송은 사용하지 않는다. HTTP 수신은 JSON 측정 데이터만 받으며 제어 API는 제공하지 않는다. 월드 ID·실행 ID·증가하는 순서 번호로 오래된 프레임과 리셋을 구분한다. 최신 월드보다 먼저 시작한 월드는 최신 Play를 덮어쓸 수 없다. 새 클라이언트 연결에는 마지막 원본 프레임과 그 프레임의 현재 신선도 상태를 함께 보낸다.

웹이 끊겨도 Unreal 물리 계산은 계속된다. 다시 서버/웹을 열면 최신 프레임을 수신한다. 측정값은 물리 모델의 ground truth이며 실제 하드웨어 센서 오차나 자율주행 기능은 아니다.

## 검증

```powershell
node Web/PhysicsLab/bridge-test.mjs
node Web/PhysicsLab/build.mjs
powershell -NoProfile -ExecutionPolicy Bypass -File Scripts/Test-WebTelemetry.ps1
```

브리지 테스트의 인위적 프레임은 별도의 임시 포트에서 프로토콜·UI 상태를 검증하는 용도다. 실제 연결 증명은 `Test-WebTelemetry.ps1`의 Unreal 실행으로 수행한다. 실제 양중·노면 마찰 변경·R 리셋을 재현하고, 서버가 수신한 원본과 웹 표시 모델의 값이 일치하는지 확인한다. 결과와 수신 원본은 `Saved/Validation/UnrealWeb-*`, `Saved/Validation/WebTelemetry-*` 아래 보관한다.

기존 물리 수치 검증 기록은 [PhysicsCalibration.md](PhysicsCalibration.md)에 유지한다.
0.13.1부터 `wheels[].solver_contact`에 실제 내부 스텝 접촉점 하중 합, 폭 방향 3구간 하중, 세계 좌표 접촉 하중 중심, 법선력 모멘트, 최대 동시 접촉점을 전송한다. 브라우저는 수신값만 표시한다. 구간 합은 점별 법선 하중 합과 일치하며 면적·압력은 새로 가정하지 않는다. [정의와 모서리 접촉 범위](TyreContact.md)를 참고한다.
0.14.0의 `wheels[].tread`에는 실제 조각별 반경 변형·속도·하중·세계 위치, 탄성 에너지와 스트로크 상태가 포함된다. 웹의 막대는 수신한 압축량만 표시한다. [분할 모델과 측정 정의](DiscreteTread.md)를 참고한다. 기존 단일 수직 스프링은 새 모드에서 꺼지며 비교 모드에서만 유지한다.

0.13.0 부품 상세 프로필에서는 포크 구간 변형·재료 응력 추정·잔류 힌지각·연결 파손, 이동 마스트·체인 장력/신장, 오일 온도·동점도·펌프 압력·배관 압력 절점과 점성 손실을 추가 전송한다. 타이어 전단 변형과 추정 패치/면압도 원본에 포함한다. 접촉 패치·면압은 강체/구성식에서 산출한 추정치이며 실제 접촉 압력 분포 계측과 구분한다. [부품 모델과 출처](RealComponentPhysics.md)를 참고한다.

### 2026-10-25 실제 연결 확인

Unreal 빌드 성공 후 `UnrealWeb-20261025-221926-938` 실행에서 3/3 통과했다. 양중 258개, 마찰 변경 126개, 리셋 234개 원본 프레임을 수신했다. 양중 높이 0.0085→0.8974 m, 유지 압력 2.803166 MPa를 확인했고, 원본의 포크 반력·압력·바퀴 하중 합이 웹 표시 모델과 일치했다. 노면 μ 0.8→0→0.8의 실제 접촉 μ 변경, 리셋 후 새 실행 ID와 기록 초기화, Play 종료 상태도 확인했다. 별도 브리지 프로토콜 검증은 23/23 통과했다.


### 0.17 메카넘 묶음 1–4

`drive_update=physics_inner_step`, `actual_realtime_ratio`, `physics_clock_lag_s`, `controls.auto_hold`를 실제 실행에서 전송한다. 월드 최대 간격은 1/30초이며 느린 프레임에서는 진행이 느려진다. Space 입력과 유효 바퀴 브레이크 한도는 구별한다. AUTO HOLD는 무입력 저속에서 축 브레이크를 적용하며 N 중립에서 해제된다.

`wheels[].mecanum.contact_geometry=inscribed-9-spheres`, `slip_measurement=physics-contact-normal-impulse-weighted-rms`는 현재 접촉 모델과 슬립 정의다. 접촉 슬립은 같은 내부 스텝의 실제 접촉점 상대속도를 집계한다.

에너지의 `shaft_joint_work_j`와 `shaft_loss_j`는 부호를 보존한다. 음수 소산 잔차를 실제 발열로 해석하지 않으며 `full_closure=false`다. 관측식·전송 검증은 물리 보존 검증과 구별한다. [현재 결과와 잔여](MecanumBatch1234.md)를 참고한다.

`Start.bat`는 ForkliftSim 전용 기본 포트 4174를 사용하며 PortSim의 4173과 분리한다. 4174가 다른 프로젝트의 포트이면 4175부터 빈 포트를 자동 선택하고 Unreal 전송 주소를 일치시킨다. 명시한 `-WebPort`는 그대로 사용한다.


## 0.18 수치 상태와 실제 CSV

기본 메카넘 접촉 계산은 1920Hz·32/32회다. `bearing_integrator=symmetric_spin_verlet`, `physics_selected_step_s`, `physics_smallest_observed_s`, `physics_inner_clock_error_s`를 Unreal에서 전송한다. 관측 최소 간격은 실제 내부 콜백 횟수와 출력 구간으로 계산한 평균이며, 선택 간격과 구별한다.

`energy_audit.bearing_transport_work_j`는 내부 베어링 예측 반작용이 한 일이다. 외부 입력 에너지나 확인된 발열로 추가하지 않는다. 브레이크 기준각 수정 후 관절 순일·탄성 저장 변화로 축 손실을 관측한다. `full_closure=false`를 유지하며 접촉·구조·수치 소산의 완전한 분리는 미완료다.

웹 내보내기는 기존 열 뒤에 새 필드 5개를 추가해 메카넘 12롤러 구성에서 **179열**이다. 실제 높은 적재 운반 세션 600프레임으로 전송·부품·접촉·수지와 내보내기 함수의 열/값 대응을 검사했다. 함수 검사는 브라우저 다운로드 자동화와 구별한다. `Web/PhysicsLab/verify-conservation.mjs`가 이 검사를 수행한다. [전체 검증](MecanumConservation.md).


2026-10-29 물성/에너지 계측 보강: `energy_audit.chain_damping_j`는 기준 이후 체인 구성식 손실이고 유압유 발열과 별개다. `wheels[].mecanum`의 `physical_wheel_mass_kg`, `hub_inertia_mass_frame_kg_m2`, `contact_inner_peak_n`, `catalogue_*`는 실제 Unreal 바디와 접촉에서 온 값 및 명시적인 비교 메타데이터다. 롤러 `inertia_mass_frame_kg_m2`와 `mass_frame_rotation_xyzw`는 SI 주관성 및 컴포넌트에 대한 질량 프레임 회전이다. `contact_inner_peak_n`은 해당 수신 물리 구간 내부 스텝의 최대값이며 프레임 평균 `normal_n`과 다르다. `rated_load_qualified=false`, `full_closure=false`를 유지한다. 웹 기록 CSV는 192열이다. 실행은 `Start.bat` → Play.
