# HW_06 Hybrid A* Algorithm & Trajectory Generation

## Assignment : Hybrid A* Algorithm

- 과제 목표
    Hybrid A* 알고리즘은 기존 A* 알고리즘에서 차량 거동에 따라 셀 내의 차량의 위치 및 Heading을 고려하여 실제 차량이 주행 할 수 있는 최적의 경로를 만들어 내는 알고리즘입니다.
    차량의 Dynamic 특성을 고려한 알고리즘이지만 A* 알고리즘과 달리 최적의 경로를 선택하지 못하거나 또는 경로를 전혀 못찾을 수 있습니다.


(1) Find next State

    def expand(self, current, goal):
        g = current['g']
        x, y, theta = current['x'], current['y'], current['t']

        # The g value of a newly expanded cell increases by 1 from the
        # previously expanded cell.
        g2 = g + 1
        next_states = []

        # Consider a discrete selection of steering angles.
        for delta_t in range(-35,40,5):
            # TODO: implement the trajectory generation based on
            # a simple bicycle model.
            # Let theta2 be the vehicle's heading (in radian)
            # between 0 and 2 * PI.
            # Check validity and then add to the next_states list.
            d2r = delta_t*np.pi/180.0
            omega = self.speed / self.length *np.tan(d2r)
            next_x = x + self.speed * np.cos(theta)
            next_y = y + self.speed * np.sin(theta)
            next_theta = theta + omega
            if next_theta >= 2*np.pi:
                next_theta -= 2*np.pi
            elif next_theta < 0:
                next_theta %= 2*np.pi
            next_g = g + 1
            next_f = next_g + self.heuristic(next_x, next_y, goal)
            next_s = {
                     'f': next_f,
                     'g': next_g,
                     'x': next_x,
                     'y': next_y,
                     't': next_theta,
                     }
            next_states.append(next_s)
            

        return next_states

 - expand 함수는 현재 State에서 지정된 조향 각 에 따라 다음 time_step에서의 State(차량의 위치, Heading)을 반환합니다.
 - 조향각은 [-35deg, 35deg]에서 5deg 간격으로 지정되었으며 Bicycle-Model에 따라서 차량의 State가 결정 됩니다.
 - 차량의 Yaw값은 [0, 2pi]로 Norimalize 해주었습니다.


(2) Search Function

    def search(self, grid, start, goal):
        # Initial heading of the vehicle is given in the
        # last component of the tuple start.
        theta = start[-1]
        # Determine the cell to contain the initial state, as well as
        # the state itself.
        stack = self.theta_to_stack_num(theta)
        g = 0
        s = {
            'f': self.heuristic(start[0], start[1], goal),
            'g': g,
            'x': start[0],
            'y': start[1],
            't': theta,
        }
        self.final = s
        # Close the initial cell and record the starting state for
        # the sake of path reconstruction.
        self.closed[stack][self.idx(s['x'])][self.idx(s['y'])] = 1
        self.came_from[stack][self.idx(s['x'])][self.idx(s['y'])] = s
        total_closed = 1
        opened = [s]
        # Examine the open list, according to the order dictated by
        # the heuristic function.
        while len(opened) > 0:
            # TODO: implement prioritized breadth-first search
            # for the hybrid A* algorithm.
            opened.sort(key=lambda s : s['f'], reverse=True)
            curr = opened.pop()
            x, y = curr['x'], curr['y']
            if (self.idx(x), self.idx(y)) == goal:
                self.final = curr
                found = True
                break

            # Compute reachable new states and process each of them.
            next_states = self.expand(curr, goal)
            for n in next_states:
                x2 = n['x']
                y2 = n['y']
                theta2 = n['t']
                if not (0 <= y2 < grid.shape[0] and 0 <= x2 < grid.shape[1]):
                    continue
                
                stack_num = self.theta_to_stack_num(theta2)
                
                if self.closed[stack_num][self.idx(x2)][self.idx(y2)] == 0 and grid[self.idx(x2)][self.idx(y2)] == 0:
                    opened.append(n)
                    self.closed[stack_num][self.idx(x2)][self.idx(y2)] = 1
                    self.came_from[stack_num][self.idx(x2)][self.idx(y2)] = curr
                    total_closed +=1
                
                
        else:
            # We weren't able to find a valid path; this does not necessarily
            # mean there is no feasible trajectory to reach the goal.
            # In other words, the hybrid A* algorithm is not complete.
            found = False

        return found, total_closed

 - Search Function은 A* 알고리즘을 통해 경로를 탐색하는 역할을합니다.
 - Heuristic Function은 Euclidean distance를 사용하였으며 Heruistic에 대한 코드는 다음과 같습니다.

       def heuristic(self, x, y, goal):
           # TODO: implement a heuristic function.
        
           return math.sqrt((goal[0] - y)**2 + (goal[1] - x)**2)

 - 위에서 Expand 메서드로 부터 얻은 차량의 다음 State가 Map상에서 실제 주행 가능한 영역인지 판단하고 Goal까지 최적의 경로를 탐색합니다.
 - theta_to stack_num 메서드는 입력된 theta의 Index를 찾아주는 함수로 코드는 다음과 같습니다.

       def theta_to_stack_num(self, theta):
           # TODO: implement a function that calculate the stack number
           # given theta represented in radian. Note that the calculation
           # should partition 360 degrees (2 * PI rad) into different
           # cells whose number is given by NUM_THETA_CELLS.
           theta1 = (theta + 2*np.pi)%(2*np.pi)
           stack_num = round(theta1*self.NUM_THETA_CELLS/2*np.pi)%self.NUM_THETA_CELLS
           return stack_num

 - 위에서 찾은 theta Index를 통해 탐색하는 Cell의 층수에 대한 경로를 탐색하여 목적지까지 최적의 경로를 찾아내는 알고리즘입니다.
 - 하지만 Map을 얼마나 세분화 할것인가 (Granularity) 또는 차량의 Heading을 얼마나 세분화해서 볼것인가(NUM_THETA_CELL)에 따라 최적의 경로를 선택하지 못하거나 경로를 찾지 못할 수 있습니다.
