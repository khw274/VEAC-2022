# 2022년 동계 혁신공유대학 가상환경 기반 자율주행 경진대회

● 가상환경(MORAI 시뮬레이션)에서 도로주행, 장애물 및 신호등 인식 자율주행 미션 수행  
● ROS 환경에서 자율주행 알고리즘 프로그래밍  
● 3차원 가상 주행로에서 트랙 기반 자율주행 경진대회

```본선 탈락```
## 역할
● 라이다 센서를 활용한 장애물 인지 및 회피 알고리즘 설계    
● ROS 기반 교통 정보 인지 및 정차 판단 알고리즘 설계    
● 원형 교차로 통과 판단 알고리즘 설계

## 활용
PYTHON, MORAI SIM, ROS1 noetic, UBUNTU 20.04, RVIZ, WSL

## 교육
● Python 기초 교육  
● 자율주행 프로그래밍 교육 제공(ROS, MORAI 시뮬레이션 활용법, GPS 센서, 경로 추종, OpenCV 등)  

## 대회 준비
### 초기 세팅
● 노트북 용량 문제와 편하게 윈도우와 사용하기 위해 WSL에 UBUNTU 20.04 환경을 구축함.  
● 기본적으로 UBUNTU 20.04와 호환되는 ROS1 Noetic 버전을 설치함.

### 경로 설정
MORAI SIM 수동모드로 설정

path_maker 노드를 실행시켜 주어진 경로 폴더에서 파일로 저장할 차량의 위치 정보를 받아옴.  
수동으로 일정 거리 이상 이동할 때마다 위치를 기록하고 파일에 작성, 세밀하게 조작해 최대한 깔끔한 경로를 추출함.

![image](https://github.com/khw274/VEAC-2022/assets/125671828/019d9230-e1f3-47a6-8ae0-34f3928cc3e9)  

더 정확한 경로 추종을 위해 Pure Pursuit 변수값 조정 > 다음에 분석

### 미션 코드 설계
#### (속도 조정 방법)
wecar planner(메인 노드)에서 계획한 현재 속도, 각 미션을 수행할 때 필요한 적정 속도 모두 다르기 때문에 때에 따라 속도값을 조절하는 것은 필수적이다.

속도값 조정을 위해 사용한 방법을 단계별로 설명하도록 하겠다.
- 먼저 상황에 따른 속도값을 저장하기 위한 비어있는 리스트를 생성한다.
- 각 미션에서 publish 한 속도 변수들을 충분히 큰 수로 초기화한다.(999로 초기화 하였다)
- 속도 리스트 중에서 가장 작은 속도 값을 별도의 변수에 저장하게 하고, 큰 값으로 초기화 해놓은 속도들 중 사용할 속도만 작은 값으로 설정한다.  ex) min_vel = min(속도 리스트)
  해당 작업을 거치면 사용할 속도(가장 작은 속도 값)만 불어오게 되어 적재적소에 활용할 수 있다.
- 특정 속도 값을 사용하지 않을 시에는 999로 초기화(적합한 다른 속도 값을 불러오기 위하여)
  특정 속도로 주행하고 싶을 땐 나머지 속도 값을 999로 초기화하고 특정 속도를 그보다 작은 값으로 설정해 min_vel(가장 작은 속도 값)을 불러와 속도를 조정하였다.

이렇게 적합한 속도를 저장하는 min_vel을 차량 모터의 RPM으로 변환하여 motor_msg에 저장하는 과정을 걸쳐 차량의 속도를 최종적으로 제어할 수 있었다.  

```python
 # 속도 Filtering
spd_list = []
spd_list.append(self.cc_vel) # wecar planner 계획한 현재 속도
spd_list.append(self.traffic_vel)
spd_list.append(self.dynamic_vel)
spd_list.append(self.rotary_vel)
spd_list.append(self.stop_vel)


# 속도 리스트 중에서 "최솟값"
min_vel = min(spd_list)

self.motor_msg = min_vel * self.rpm_gain / 3.6
```
#### (신호등 정차 미션)
신호등 미션에서는 카메라를 사용하지 않는 방법을 채택했다.

MORAI에서 정의한 메시지 유형을 포함한 패키지 morai_msgs에서 GetTrafficLightStatus 메시지를 불러왔다. 

GetTrafficLightStatus 메시지는 실시간으로 전송되는 MORAI SIM에서의 신호등 상태 정보를 포함하고 있다. 

- /GetTrafficLightStatus, /current_waypoint 토픽을 subscribe 하여 신호등 상태와 현재 차량의 waypoint 정보를 받아옴
- 변수에 신호등 상태에 따른 output 값 저장(파란불일 시 '16' 값이 나오는 것을 확인)
- MORAI SIM에서 신호등 구간을 임의로 정해 waypoint 정보 저장(측정 결과 171~172 신호등 웨이포인트), rosrun echo 명령어를 통해 /current_waypoint 토픽에서 발행된 메시지를 확인하여 측정함 
- 신호등 상태에 따른 if문 작성, 파란불이 아닐 시 속도를 0으로 지정함
- 설정한 속도를 변수에 저장하고 Publisher을 통해 메인 노드가 Subscribe할 수 있도록 메시지 송신
<img src="https://github.com/khw274/VEAC-2022/assets/125671828/90d58a0f-d136-4eae-850a-995858a92341" width="700" height="400"/>

#### (로터리 미션)
##### WAYPOINT 지정
우선 로터리 미션 구간을 정하기 위해 waypoint 정보를 저장하였다. waypoint는 세 지점을 측정해야 하는데   
```1. 장애물이 근접해오는 지점, 2. 로터리를 들어가기 전 지점, 3. 로터리에 진입했을 때부터 나갈 때까지의 지점``` 이다.

장애물이 근접해오는 지점을 체크하는 이유는 장애물이 차량에 상당히 근접했을 때 로터리에 진입하게 되면 충돌할 가능성이 있기 때문이다.

##### Topic 정보
장애물과 차량의 거리를 조절해 속도를 제어하는 방법을 사용하기로 했다.

따라서 총 4개의 토픽을 Subscribe 했다.

1. 차량 주변 환경의 거리 정보를 수집하는데 사용되는 레이저 스캐너 데이터(/laser2pcd_map 토픽)
2. 차량의 지역 경로(/local path 토픽)
3. 현재 차량의 상태 정보 -> 현재 차량의 위치(/Ego_topic 토픽)
4. 차량의 웨이포인트(/current_waypoint 토픽)

##### 로터리 알고리즘
##### ● 로터리에 진입 전 속도 제어
pcd_list[] 를 선언하고 리스트에 레이저 스캐너에 인식된 장애물의 위치 좌표 중 x와 y만 저장한다.(z 좌표는 사용 X)

```python
self.pcd_list = []

for i in msg.points:
    pcd = [i.x, i.y]
    self.pcd_list.append(pcd)
```
장애물의 좌표와 미리 측정해둔 장애물이 근접해오는 지점(1)의 좌표 사이 유클리드 거리를 측정한다.

차량이 로터리를 들어가기 전 지점(2)에 도달했을 때 방금 전 측정한 유클리드 거리가 1 미만이라면 장애물이 경로 안에 있다고 가정하여 차량을 정지시키고, 1 이상이면 속도를 최댓값으로 초기화시켜 정상적인 속도로 주행하게 한다.

* 여기서 거리 1은 정해진 것이 아니라 여러 번 테스트를 통해 기준을 정한 것이다. 앞으로 나올 비교 대상 거리 값도 동일하다.

로터리를 들어가기 전 지점(2)에 차량이 없다면 마찬가지로 정상 속도로 주행하도록 최댓값으로 초기화한다.

마지막으로 stop_vel을 Publish 시켜 메인 함수가 받을 수 있도록 한다.
```python
for i in self.pcd_list:
  for j in rotary_waypoints:
      dx = i[0] - j[0]
      dy = i[1] - j[1]
      dst = math.sqrt(dx * dx + dy * dy)
      
      if dst < 1:
          checking = False
      else:
          checking = True
          
      # 현재 waypoint가 213 또는 214 일 때, 장애물 여부에 따라 정지 속도 설정
      if 213 <= self.current_waypoint <= 214:
          if checking == False:
              self.stop_vel = 0
          elif checking == True:
              self.stop_vel = 999
      else:
          self.stop_vel = 999
  
self.stopvel_pub.publish(self.stop_vel)
```
##### ● 로터리에 진입 후 속도 제어
로터리에 진입 후 전방에 있는 동적 장애물의 속도와 거리에 따라 차량의 속도를 조절해 충돌하지 않도록 제어해야 한다.

우선 차량과 가까운 범위 내의 경로를 나타내는 지역 경로 좌표와 장애물의 위치 좌표 사이 유클리드 거리를 구해 그 거리가 0.4 미만일 시 obstacle_list(장애물 리스트)에 추가하였다. 이는 차량의 지역 경로 내에 장애물이 근접했다는 의미이다.

장애물 리스트에 추가된 장애물의 좌표와 현재 차량의 좌표 사이 유클리드 거리를 계산해 만약 로터리에 진입했을 때부터 나갈 때까지의 지점(3) 내에 있을 때 방금 계산한 차량과 장애물 사이 거리가 0.5 미만일 시 차량을 정지시키고 그 외 지점은 정상 속도로 주행하도록 한다.

마지막으로 rotary_vel을 Publish 시켜 메인 함수가 받을 수 있도록 한다.

```python
obstacle_list = []
for i in self.pcd_list:
    for j in self.local_path_list:
        dx = i[0] - j[0]
        dy = i[1] - j[1]
        dst = math.sqrt(dx * dx + dy * dy)
        if dst < 0.4:
            obstacle_list.append(i)

for i in obstacle_list:
    dx = i[0] - self.ego_x
    dy = i[1] - self.ego_y
    dst = math.sqrt(dx * dx + dy * dy)
    
    # 현재 waypoint가 215부터 236일 때, 거리에 따라 rotary_vel 설정
    if 215 <= self.current_waypoint <= 236:
        if dst < 0.5:
            self.rotary_vel = 0
    else:
        self.rotary_vel = 999

self.vel_pub.publish(self.rotary_vel)
```
#### (동적 장애물 미션)

#### (정적 장애물 미션)

#### (라바콘 미션)
