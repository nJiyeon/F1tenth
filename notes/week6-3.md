# 📝 주간 회의록 - WEEK 6

- **회의 일시**: 2025-10-17 (금) 19:00 ~ 20:30
- **회의 장소**: [대면 미팅] 공대 9호관 5층
- **참석자**: 윤우린, 남지연, 이태웅 학사과정생
  
---

### 📍 주요 논의 
### bash 1 - ~/f1tenth_ws

```bash
ros2 launch f1tenth_stack setup_bringup.py
```

### bash 2 (로컬 컴퓨터 내)

```bash
rviz
```

### bash 3 - ~/f1tenth_ws

```bash
ros2 launch particle_filter localize_launch.py
```

### bash 4 - ~/f1tenth_ws

```bash
slam 
```

결론

: steering 값을 받아오는건 해결됨

(완벽한지는 모름)

map을 따는 것부터 문제임 !

### 월요일

- 다 같이 돌려보기

### 화요일

- 알파시티 가보기

ros2 run waypoint_logger waypoint_logger --ros-args -p odom_topic:="/pf/viz/odom
