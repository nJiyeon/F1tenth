이전에 실습했던 wall_following패키지를 보완하여 구현함

**문제점**
1. 처음으로 진행했을 땐 왼쪽 벽만 인식하여 오른쪽 벽에 딱 붙어서 가는 문제점발생
→ 양쪽벽을 동시에 사용해 중앙 추종옵션을 구현하여 해결

1. 벽에 과도하게 붙어 L혹은 R이 range_min/range_max에 몰려 잘못된 오차발생→유효성 검사/폴백을 사용하여 해결
ex)왼쪽만 valid상태일때 single-left로 전환

Kp: 오차가 클수록 큰 제어값을 즉각적으로 반영, 반응 속도 ↑, 하지만 너무 크면 **진동 발생**

Ki:누적된 오차를 보정 → 오랫동안 남는 편차 제거, 너무 크면 느려지거나 불안정

Kd:오차 변화율을 고려 → 급격한 변화를 완화, overshoot 억제, **진동 감소** 효과

```bash
ros2 run wall_following_pkg wall_follower_node --ros-args \
  -p mode:=dual -p center_offset:=0.0 \
  -p kp:=0.85 -p ki:=0.001 -p kd:=0.20 \
  -p lookahead:=0.8 -p velocity:=1.0 \
  -p phi_deg:=55.0 -p theta_deg:=8.0 -p max_wall_dist:=8.0

```

위의 명령어를 사용하여 주행을 시킴

kp,ki,kd값을 변경해보고 주행 결과를 토대로 최적의 값을 찾아서 구현

p:약 0.85가 적당

I: 0.001

d:0.2
정도로 구현함


**최종결과**
<img width="1690" height="1014" alt="image" src="https://github.com/user-attachments/assets/bc74144b-1386-4bb3-aa81-69abcd2ffd94" />

<img width="966" height="614" alt="image" src="https://github.com/user-attachments/assets/104828ec-1cae-4913-b775-6b1919720560" />

