# 📝 주간 회의록 - WEEK 7

- **회의 일시**: 2025-10-21 (화) 19:00 ~ 21:00
- **회의 장소**: [대면 미팅] 공대 9호관 5층
- **참석자**: 윤우린, 이태웅, 남지연, 김시후 학사과정생, 최윤도 박사과정생

---

### 📍 주요 논의 요약
<aside>


### 실습 순서

1. **RTCAR 와이파이에 노트북 연결**
2. **CTRL+O, CTRL+E 해서 터미널 여러개 한번에 SSH 연결**
3. [**bringup.py](http://bringup.py) 실행**
4. **Rviz2 실행**
    1. add : laserscan
    2. add : fixed 뭐시기 → laser
    
    ⇒ 지지직 거리는거 확인
    
5. **SLAM 실행**
    1. 터미널에 그냥 **`slam`** 치면 알아서 실행됨
    2. 주행하면서 맵 따기 (곡선에서 특히 저속 주행)
    3. **SLAM 실행과 다른 터미널에서 저장 명령어**로 맵 저장
        1.  history에 있는 -f test.knu 이 명령어로 파일 이름 바꿔서 map 저장
    4. 저장된 맵 확인 (.yaml, .pgm 모두 생성됨)
    5. SLAM 끄고
6. **맵 경로 수정 (생성한 맵을 particle filter로 이동 시키는 과정)**
    1. 터미널1 : config -> yaml파일 nano편집 -> map이름 변경후 저장
    2. 터미널2 : colcon build & setup.bash 실행
    3. **맵 경로 수정** : particule filter/src/maps 여기다가 cp ~/251014.* .
7. **Particle filter 실행**
    1. rviz2 add : map
    2. rviz2 add : pose array
8. **Waypoint 생성 (pure pursuit이 따라갈 점 생성)**
    1. waypoint logger를 실행한 후 주행 시작**(particle filter가 켜진상태로)**
    2. ^C를 통해 로그파일 저장(날짜.csv형태로 저장)
    3. log가 찍힌 csv파일을 f1tenth_ws/src/purepursuit/waypoints로 복사
    4. f1tenth_ws/src/purepursuit/config 내 f1tenth_ws/src/purepursuit/config 에 있는 yaml파일 코드 경로 수정 
9. **Pure pursuit 실행**
    1. ros2 launch pure_pursuit launch.py로 실행
    2. **파라미터 수정 or waypoint 생성을 촘촘히 또는 멀리 해보면서 조정**
</aside>

### 실습 결과

- 계속해서 방향을 못잡는 것 같음
    - 맵 따는 과정에서 생긴 문제인지 코드 자체의 문제인지 여쭤볼 것
    - 라이다 센서가 유리벽에 의해서 경계를 잘 못잡는 것인지 여쭤볼 것

- 4층 DIY실 예약

### 논문

- 3페이지 분량으로 작성하자
- 오토웨어 MPPI 플래너 구현하고 돌려봤다
    - 오토웨어 MPPI는 어떤 거고~~ 차량에 돌리기 위해서는 ~~~ 이렇게 튜닝이 필요했고
    - ~~~~ 이런 것들을 해본 결과 잘 돌아갔다
    - pure pursuit이랑은 이런이런 차이가 있었다
    - 비교도 힘들면 이런 거 했다 정도로만 작성하면 될듯 ~~~!!!

  정리하자면,,

  : 오토웨어에서 MPPI 트레젝토리를 지원하는데 실제 차에다가 테스트를 할 수 없으니까 경량화된 플랫폼인 f1tenth에서 돌려봤다. 성능은 이정도로 나왔다.  (수식 2개 정도만 작성하기 + 수식 설명도 작성하기)

  1. 서론
  2. 본론
  3. 결론

### 역할

- 2명은 알고리즘
- 1명은 논문 작성
- 1명은 ROS

### 대회 이후에 할 일

- 부산대 자율주행 대회

### 포스터 발표, oral 발표..?? 온라인 발표
