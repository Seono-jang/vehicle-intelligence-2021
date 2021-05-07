# HW_04 Path Planning & the A* Algorithm

## Assignment

- 과제 목표
    이번 과제의 목표는 dynamic programming을 기반으로 한 path planning 알고리즘입니다.
    이전 예제와의 차이점은 차량의 dynmic 특성을 고려하고 직진/좌회전/우회전의 각각의 Action마다 Cost를 다르게 설정해주고 이에 대하여 최적화된 경로를 찾아내는 것이 목표입니다.

(1) Value Function / Policy Function
 
 - 경로 생성 알고리즘에서 Value 값과 Policy를 계산하는 부분입니다.




        for y, x, t in p:
            # Mark the final state with a special value that we will
            # use in generating the final path policy.
            if (y, x) == goal and value[(t, y, x)] > 0:
                # TODO: implement code.
                value[(t,y,x)] = 0
                policy[(t,y,x)] = -999
                change = True

  

 - for 문을 통해서 맵 상의 각각의 좌표값을 받아와 반복문으로 실행하여 각 좌표에 대한 Value값과 Policy를 계산합니다.
 - 먼저 좌표 y,x 가 목적지에 도착한다면 Value값을 0으로 설정해 주어 목적지에서의 Cost가 0이 되도록 만들어 줍니다.
 - 목적지의 Policy는 경로 탐색이 끝났다는 것을 의미 하기위해 임의로 -999의 값을 설정해주었습니다.


            elif grid[(y, x)] == 0:
                # TODO: implement code.
                for i in range(len(action)):
                    t2 = (t + action[i]) %4
                    y2 = y + forward[t2][0]
                    x2 = x + forward[t2][1]
                    
                    if 0 <= y2 < grid.shape[0] and 0 <= x2 < grid.shape[1] \
                        and grid[(y2, x2)] == 0:
                            v2 = value[(t2, y2, x2)] + cost[i]
                            
                            if v2 < value[(t,y,x)]:
                                value[(t,y,x)] = v2
                                policy[(t,y,x)] = action[i]
                                change = True


 - for 문을 이용하여 좌회전 직진 우회전의 Action들이 취해졌을때의 Heading과 (y,x) 값을 계산합니다.
 - Heading 값을 상하좌우 4방향으로 표기하기 위해 Action 후 Heading에 대한 값인 t2를 기존 Heading 값에 Action에 대한 Heading 변화량을 더해 준 후 4에 대한 나머지 값으로 나타내었습니다.
 - 좌표 (t,y,x)에서의 Cost와 Action에 대한 Cost를 합하여 Action을 취한 후의 최종 Cost를 산출합니다.
 - for 문을 통해 가장 Cost를 최소화 할 수 있는 Action을 찾아 Policy 각 좌표에서 취할 수 있는 최적의 행동을 업데이트해 줍니다.

(2) 최적 경로 생성


    # Now navigate through the policy table to generate a
    # sequence of actions to take to follow the optimal path.
    # TODO: implement code.
    y,x,t = init
    
    while policy[(t,y,x)] != -999:
        for i in range(len(action)):
            if policy[(t,y,x)] == action[i]:
                t2 = t + action[i]
                policy2D[(y,x)] = action_name[i]
        
        y += forward[t2][0]
        x += forward[t2][1]
        t = t2
        
    if policy[(t,y,x)] == -999:
       policy2D[(y,x)] = '*'
       
    


 - 3차원으로 계산된 Policy를 2차원 평면에서의 경로를 나타내기 위한 알고리즘입니다.
 - 도착지 전 까지의 Policy2D값을 계산하기 위해 while문을 사용하였으며 (t,y,x)의 Policy값을 받아와 Policy2D에 투영하고 Action에 따른 Heading값을 구하여 Heading 방향으로 진행 후 좌표를 나타내었습니다.
 - 이후 마지막 목적지를 표기하기 위해 Policy2D 값에 '*'을 표기하도록 하였습니다.
