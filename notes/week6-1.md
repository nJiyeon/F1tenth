# 📝 주간 회의록 - WEEK 6

- **회의 일시**: 2025-10-14 (화) 19:00 ~ 20:30
- **회의 장소**: [대면 미팅] 공대 9호관 5층
- **참석자**: 윤우린, 남지연, 이태웅, 김시후 학사과정생, 최윤도 박사과정생
  
---

### 📍 주요 논의 요약
1. 저번 멘토링때 배운내용 복습
(센서브링업, ssh연결, 맵 생성,맵 복사)

터미널 총 3개 필요

<SLAM>

- 터미널1 : ssh 연결 -> [nvidia@192.168.1.103](mailto:nvidia@192.168.1.103) -> 비밀번호
=> ros2 launch f1tenth_stack [bringup~~.py](http://bringup~~.py/) 실행
- 터미널2 : rviz2
=> add : lasercan
=> map -> laser
- 터미널3 : ssh연결
=> ros2 launch slam_toolbox ~~ (history보고 복사)
=> history에 있는 -f test.knu 이 명령어로 파일 이름 바꿔서 map 저장

=> SLAM 끄고
<Particule filter>

- 터미널1 : config -> yaml파일 nano편집 -> map이름 변경후 저장
- 터미널2 : colcon build & setup.bash 실행
- particule filter/src/maps 여기다가 cp ~/251014.* .
: 하면 파일 옮겨짐
- rviz에서 pose array 추가해서 차 돌려보면 잘 따라가는지 체크

2. 쓰로틀 연습
3. 배터리 교체는 센서 다 off 한 후 꺼야함.
배터리는 어댑터에서 스펙 맞추고 충전해야함. 전압안맞추면 폭발
