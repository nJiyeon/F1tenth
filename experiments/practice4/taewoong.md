기본적인 pursuit_node.py코드를 작성한 후 L의 값을 변화시키며 실험을 진행함

```bash
ros2 run pure_pursuit_pkg pure_pursuit_node --ros-args \
-p lookahead_distance:=2.0 \
-p odom_topic:=/ego_racecar/odom
```

lookahead_distance값을 바꿔가며 주행을 비교함

**문제점**
gym_ros패키지와 연동 시에 경로에 대한 값, publisher와 subcribtor가 일치하지 않아서 해결하는데 시간을 소비함
→ 실행중인 토픽을 확인해 일치시킨 후 재실행 시켜 해결

과제-2 결과: L값이 작을 수록 곡선을 잘따라가지만 L값이 클 땐 곡선경로를 잘 반응하지못함

이에 따라 advanced버전으로 곡률에 따라 속도를 조절할 수 있는 기능을 추가해 새로운 버전을 제작함

곡률이 클수록 속도를 낮추고 곡률이 작을수록(직선경로) 속도를 높이도록 설계함


**결과**
<img width="1594" height="796" alt="image" src="https://github.com/user-attachments/assets/0a109860-fe55-44df-a679-d34ad90005b7" />

곡률이 0.87인 구간에서 속도가 자동으로 0.69로 줄어듬→ 직선에서 빠르게 달리다가 커브구간에서 자동으로 감속하는 동작이 정상적으로 구현됨이 확인됨
