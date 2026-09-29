# pkrc_control 파라미터 레퍼런스

`keyboard_control_robust_*` 3개 노드의 파라미터 목록. rqt_reconfigure로 주행 중 조절하는 값과, 고정값으로 숨겨둔 값을 구분해 정리했다.

기본값 열은 노드별로 다를 때만 나눠 적었다 (`orig` = `keyboard_control_robust_original`, `good` = `..._update_good`, `test` = `..._update_test`). `—` 는 그 노드에 없는 파라미터.

## 실행

```bash
source install/setup.bash
ros2 run pkrc_control keyboard_control_robust_update_good
```

노드가 뜨면 rqt_reconfigure GUI가 함께 실행되고, 노드를 끄면(Ctrl+C) GUI도 함께 닫힌다. 값을 바꾸면 노드 터미널에 `실시간 반영: hh_rate_kp=3.2` 로그가 찍힌다 — 이 로그가 없으면 반영되지 않은 것이다.

GUI 없이 띄우려면 `--ros-args -p tuning_gui:=false`, 터미널에서 직접 바꾸려면 `ros2 param set /keyboard_control_teleop <이름> <값>`.

**GUI에서 바꾼 값은 소스에 저장되지 않는다.** 재시작하면 아래 기본값으로 돌아가므로, 확정한 값은 `declare_parameter`에 직접 적어야 영구 반영된다.

## Heading Hold — 목표 방위 유지 (2중 루프 PID)

바깥 루프가 각도 오차로 목표 각속도를 만들고, 안쪽 루프가 그 각속도를 추종한다.

| 파라미터 | 역할 | orig | good | test |
|:--|:--|--:|--:|--:|
| `hh_angle_kp` | 각도 오차 → 목표 각속도 변환 게인 (클수록 빠르게 돌아옴) | 3.5 | 6.0 | 6.0 |
| `hh_angle_ki` | 각도 정상상태 오차 제거 적분 게인 | — | 0 | 0 |
| `hh_rate_kp` | 각속도 오차 → yaw 출력 비례 게인 | 2.8 | 2.8 | 2.0 |
| `hh_rate_ki` | 각속도 적분 게인 (클수록 잔류 오차 빨리 없앰, 과하면 진동) | 1.8 | 0.07 | 0.03 |
| `hh_rate_kd` | 각속도 미분 게인 (진동 감쇠, 과하면 노이즈 증폭) | 0.85 | 0.95 | 0.2 |
| `hh_rate_aw` | 적분 와인드업 방지 계수 (출력 포화 시 적분 감쇠율) | — | 1.0 | 1.0 |
| `hh_max_rate` | 목표 각속도 상한 (rad/s) — 급회전 방지 | 0.8 | 0.8 | 0.8 |
| `hh_deadband_deg` | 이 각도(°) 미만 오차는 무시 — 미세 떨림 억제 | 0.5 | 0.1 | 0.1 |
| `hh_output_slew` | yaw 출력 변화율 상한 (per sec) — 급격한 출력 변화 차단 | 4.0 | 4.0 | 4.0 |

## Yaw Rate — 수동 회전 (A/D 키)

| 파라미터 | 역할 | 기본값 |
|:--|:--|--:|
| `yr_target_rate` | A/D 키를 누를 때 목표 각속도 (rad/s, 0.30 ≈ 17°/s) | 0.30 |
| `yr_kp` | 각속도 추종 비례 게인 | 2.0 |
| `yr_ki` | 각속도 추종 적분 게인 | 2.5 |
| `yr_kd` | 각속도 추종 미분 게인 | 0.10 |
| `yr_max_output` | 수동 회전 출력 상한 | 0.50 |

## 정지·릴리즈 거동

키를 뗀 직후 헤딩 홀드가 과출력으로 진동하는 것을 막는 값들.

| 파라미터 | 역할 | orig | good | test |
|:--|:--|--:|--:|--:|
| `idle_grace_sec` | 모션 종료 후 이 시간(초)까지는 출력을 그대로 유지 | 1.5 | 1.5 | 1.5 |
| `idle_output_scale` | 위 유예가 끝난 뒤 적용할 출력 배율 (0.30 = 30%) | 0.30 | 0.30 | 0.30 |
| `release_output_scale` | 키를 뗀 순간의 출력 배율 | — | — | 0.15 |
| `yaw_route_hold_sec` | 회전 종료 후 목표 방위를 재설정하지 않고 유지할 시간 | — | — | 1.5 |

## Feedforward — 커플링 선보상

한 축의 추력이 다른 축을 흔드는 것을 미리 상쇄한다. `original`에만 sway/surge → yaw 보상이 있고, 나머지 둘은 pitch → heave만 쓴다.

| 파라미터 | 역할 | orig | good | test |
|:--|:--|--:|--:|--:|
| `ff_pitch_to_heave` | pitch 기울기 → heave 보정량 | 2.5 | 2.5 | 2.5 |
| `ff_sway_to_yaw` | sway 명령 → yaw 보상 (좌우 공통) | 0.60 | — | — |
| `ff_sway_left_to_yaw` | 좌측 sway → yaw 보상 (좌우 비대칭 대응) | 0.55 | — | — |
| `ff_sway_right_to_yaw` | 우측 sway → yaw 보상 | 0.40 | — | — |
| `ff_surge_to_yaw` | surge 명령 → yaw 보상 | 0.05 | — | — |
| `ff_ramp_time_sec` | 보상값이 최대치까지 올라가는 시간 (초) | 0.40 | — | — |

## Depth — 깊이 제어 (2중 루프)

| 파라미터 | 역할 | 기본값 |
|:--|:--|--:|
| `depth_pos_kp` | 깊이 오차 → 목표 하강/상승 속도 변환 게인 | 2.0 |
| `depth_vel_kp` | 수직 속도 추종 비례 게인 | 1.2 |
| `depth_vel_ki` | 수직 속도 추종 적분 게인 (부력 상쇄) | 0.3 |
| `depth_vel_kd` | 수직 속도 추종 미분 게인 | 0.15 |
| `depth_step` | 키 1회당 목표 깊이 변화량 (m) | 0.10 |
| `depth_max_ascent_vel` | 목표 상승 속도 상한 (m/s). 부력이 추력을 도와 상승이 빨라지므로 하강 상한(0.4)과 따로 둔다 | 0.15 |

## DVL 드리프트 보정

DVL 속도를 읽어 조류·부력에 의한 표류를 상쇄한다. 주명령을 방해하지 않도록 보정량에 여러 제한이 걸려 있다.

| 파라미터 | 역할 | 기본값 |
|:--|:--|--:|
| `dvl_enabled` | DVL 드리프트 보정 사용 여부 | True |
| `dvl_drift_kp` | 속도 오차 → 보정 추력 비례 게인 | 0.8 |
| `dvl_drift_ki` | 속도 오차 적분 게인 (지속적 조류 상쇄) | 0.6 |
| `dvl_drift_max` | 보정 추력 상한 — 주명령을 덮지 않게 제한 | 0.35 |
| `dvl_drift_max_int` | 적분항 누적 상한 (와인드업 방지) | 0.5 |
| `dvl_vel_lpf_alpha` | DVL 속도 저역통과 필터 계수 (작을수록 부드럽고 느림) | 0.35 |
| `dvl_vel_deadband` | 이 속도(m/s) 미만은 노이즈로 보고 무시 | 0.03 |
| `dvl_stale_timeout` | 이 시간(초) 동안 DVL 신호가 없으면 보정 비활성 | 0.5 |
| `dvl_min_cmd` | 주명령이 이 값보다 작으면 보정하지 않음 | 0.10 |

## 수동 조종 명령 크기

| 파라미터 | 역할 | orig | good | test |
|:--|:--|--:|--:|--:|
| `surge_cmd` | 전후진 키 1회당 추력 명령 크기 | 0.80 | 0.5 | 0.5 |
| `sway_cmd` | 좌우 이동 키 1회당 추력 명령 크기 | 0.80 | 0.5 | 0.5 |

## 스러스터 출력 제한

`max_current_*`는 안전 한계값이라 주행 중 올리는 것은 권장하지 않는다.

| 파라미터 | 역할 | orig | good | test |
|:--|:--|--:|--:|--:|
| `max_current_surge` | 전후진 스러스터 전류 상한 (A) | 3.0 | 3.0 | 3.0 |
| `max_current_sway` | 좌우 스러스터 전류 상한 (A) | 3.0 | 3.0 | 3.0 |
| `max_current_heave` | 상하 스러스터 전류 상한 (A) | 3.5 | 5.0 | 5.0 |
| `thruster_gain` | 스러스터 6개별 출력 배율 (개체차·노후 보정) | — | 전부 1.0 | 전부 1.0 |

## 고정값 — GUI에 표시되지 않음

`read_only`로 선언해 rqt_reconfigure 목록에서 제외했다. 코드는 이 값들을 그대로 사용하며, 바꾸려면 소스를 고치고 재시작해야 한다.

| 파라미터 | 역할 | 기본값 |
|:--|:--|--:|
| `water_density` | 물 밀도 (kg/m³) — 압력을 깊이로 환산할 때 사용 | 1000.0 |
| `gravity` | 중력가속도 (m/s²) — 위와 동일한 환산식에 사용 | 9.81 |
| `atmospheric_pressure_pa` | 수면 대기압 (Pa) — 절대압에서 빼 수압만 남김 | 101325.0 |
| `dvl_topic` | DVL 데이터 구독 토픽 — 시작 시 1회만 구독하므로 변경 불가 | `/dvl/data` |

깊이 환산식: `깊이 = (측정압 - atmospheric_pressure_pa) / (water_density × gravity)`

## 기타

| 파라미터 | 역할 | 기본값 |
|:--|:--|--:|
| `tuning_gui` | 노드 시작 시 rqt_reconfigure 자동 실행 여부 | True |

## 튜닝 순서 (권장)

방위 유지가 불안정할 때는 안쪽 루프부터 잡는 것이 빠르다.

1. `hh_rate_kp` — 각속도 추종이 되는 최소값까지 올린다
2. `hh_rate_kd` — 진동이 보이면 올려 감쇠시킨다
3. `hh_rate_ki` — 잔류 오차가 남을 때만 조금씩 (과하면 느린 진동 발생)
4. `hh_angle_kp` — 복귀 속도가 느리면 올린다
5. 정지 후 흔들리면 `idle_output_scale`을 낮춘다

확정한 값은 `pkrc_control/keyboard_control_robust_*.py`의 `declare_parameter` 기본값에 반영한다.
