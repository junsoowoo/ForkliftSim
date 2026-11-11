# ForkliftSim — Unreal 실시간 계측 웹

Unreal에서 실행하는 지게차의 실제 물리 측정값을 보여 주는 로컬 대시보드다. 브라우저에서 별도 물리 계산이나 가상 양중을 실행하지 않는다.

## 실행

프로젝트 루트의 `Start.bat`를 실행하면 Unreal Editor와 웹이 함께 열린다. Editor에서 **Play**를 누르고 W로 팔레트에 포크를 넣은 다음 E로 올린다. 실제 포크 높이·좌우 지지력·유압·바퀴 하중·마찰력이 웹에서 갱신된다.

- 주소: `http://127.0.0.1:4174` (PortSim의 4173과 분리)
- 직접 Node 실행 시에도 기본 4174. 별도 환경 설정은 `FORKLIFTSIM_WEB_PORT`이며 공통 `PORT` 환경 변수는 사용하지 않는다.
- `Website.bat` 더블클릭 또는 `Start.bat -Website`: 웹 서버와 브라우저만 실행
- `Start.bat -NoWebsite`: Unreal만 실행
- `Start.bat -WebPort 4194`: 웹과 Unreal 송신 주소를 함께 변경
- R: Unreal 리셋 및 웹 그래프 새 실행 시작
- Space: Unreal Play 화면에 포커스가 있을 때 누르는 동안 제동. 웹에서 브레이크 입력과 바퀴별 토크 한도를 확인한다.
- 1/2, 3/4: Unreal 타이어/노면 마찰 변경

Node.js는 설치된 런타임 또는 이 PC의 Codex 제공 런타임을 자동 탐색한다. 오프라인 HTML 파일을 직접 열면 연결 안내만 나온다. 실시간 계측에는 로컬 서버가 필요하다.

## 화면

- 실시간 계측: 실제 차체 속도, 포크 높이·반력, 바퀴별 원시/필터 하중·눌림·저항 토크, 접촉 마찰력과 μ, 유압·유량·전력, 화물 상태.
- 실제 위치·조향 기반 상부 도식, 포크 지지력·유압 시간 그래프, 최근 기록 CSV 저장.
- 구현 기능: 8개 물리 시스템의 범위, 변수와 소스 파일 안내.
- 검증 기록: 기존 0.11.1 정량 시험의 저장 결과. 실시간 측정과 구분해 표시한다.

연결 전 값은 `—`이며, 일시정지·Play 종료·서버 끊김 시 마지막 수신값이라는 상태를 표시한다. 도식은 계측 표시이며 Unreal 영상 스트리밍은 아니다.

## 데이터 경로

`AStabilityLab / ContactLoadSampler → WebTelemetry.cpp → HTTP POST /api/telemetry → bridge.mjs → SSE /events → live.mjs`

Unreal이 최대 20 Hz로 비동기 전송한다. 웹은 SI 원본값의 단위 변환·표시·그래프만 담당한다. 서버는 루프백에만 바인딩하고 Unreal 제어 API를 제공하지 않는다. 서버가 없어도 물리 계산은 계속된다.

일반 `Start.bat`가 송신 옵션을 자동 추가한다. 직접 Unreal을 실행한다면 `-WebTelemetry=http://127.0.0.1:4174/api/telemetry`가 필요하다.

## 개발과 검증

```powershell
node Web/PhysicsLab/bridge-test.mjs
node Web/PhysicsLab/build.mjs
powershell -NoProfile -ExecutionPolicy Bypass -File Scripts/Test-WebTelemetry.ps1
```

`dist/live.html`, `live.mjs`, `live-state.mjs`, `styles.css`, `data.mjs`가 제공되는 웹 소스다. `build.mjs`는 시작 문서와 오프라인 연결 안내를 갱신한다. 예전 독립 계산기 파일은 서버에서 제공하지 않는다.

브리지 시험은 임시 포트의 명시적인 시험 프레임을 사용한다. 실제 연동 검증은 Unreal 양중·마찰 변경·리셋 실행에서 수신한 값과 웹 표시 모델을 비교한다. 원본과 결과는 `Saved/Validation`에 보관한다.

상세 측정 정의와 한계는 [LiveTelemetry.md](../../Docs/LiveTelemetry.md)를 참고한다. 외부 사이트 배포는 수행하지 않았다.

2026-11-06 자동화 5묶음: 실제 `mission` 텔레메트리에서 단계·정지 사유·추정 위치·계획 경로·재계획 횟수를 표시한다. `mission.mjs`는 수신 이력만 30열 CSV로 내보낸다. 경로 그림은 등록 접근 위치·실제 계획점·측정 위치로 그린 도식이며 Unreal 영상이나 창고 전체 지도는 아니다. 17번 기능 카드와 별도 임무 CSV 버튼을 추가했다. 사용자 실행은 여전히 루트 `Start.bat` → Play다. [임무 조작·검증·한계](../../Docs/MissionNavigation.md)를 참고한다.

`verify-mission-live.mjs`는 최종 8개 Unreal 임무 조건의 실제 SSE를 별도 기록한다. 웹 화면의 현재 세계 최근 30초·최대600프레임 이력과 이 장기 검증 기록을 구분한다. 합성 브리지 프로토콜·표시 모듈 시험은 물리·GPU·DOM 다운로드 검증으로 계산하지 않는다.


### v0.18 actual conservation telemetry

The live page reads Unreal's `bearing_integrator`, selected/observed physics step and inner clock error, plus signed `energy_audit.bearing_transport_work_j`. The qualified contact profile uses 1920 Hz and 32/32 iterations. Complete energy closure stays false. No browser-side vehicle dynamics replaces native values.

`verify-conservation.mjs http://127.0.0.1:4174` checks a stopped native session and runs the actual export handler's data construction against received history (179 columns for 12 rollers). It does not automate a browser download. Use the actual port selected by root `Start.bat`; 4173 may belong to another project. See `Docs/MecanumConservation.md` for native qualification and limits.


2026-10-29 물성/에너지 계측 보강: `energy_audit.chain_damping_j`는 기준 이후 체인 구성식 손실이고 유압유 발열과 별개다. `wheels[].mecanum`의 `physical_wheel_mass_kg`, `hub_inertia_mass_frame_kg_m2`, `contact_inner_peak_n`, `catalogue_*`는 실제 Unreal 바디와 접촉에서 온 값 및 명시적인 비교 메타데이터다. 롤러 `inertia_mass_frame_kg_m2`와 `mass_frame_rotation_xyzw`는 SI 주관성 및 컴포넌트에 대한 질량 프레임 회전이다. `contact_inner_peak_n`은 해당 수신 물리 구간 내부 스텝의 최대값이며 프레임 평균 `normal_n`과 다르다. `rated_load_qualified=false`, `full_closure=false`를 유지한다. 웹 기록 CSV는 192열이다. 실행은 `Start.bat` → Play.
