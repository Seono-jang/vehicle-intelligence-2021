# HW_05 Prediction & Behaviour Planning

## Assignment 1 - Gaussian Naive Bayse

- 과제 목표
    GNB(Gaussian Naive Bayse)를 활용한 Prediction 알고리즘은 수집된 데이터를 활용하여 [좌회전/차선유지/우회전] 상황에 대한 [s/d/s_dot/d_dot] 값에 대한 평균값과 분산값을 구하고 이 학습 데이터를 활용하여 Input Data가 어떠한 Action을 취하고 있는지 예측하는 알고리즘 입니다.



(1) Trainning Algorithm

 - 주어진 학습 Data 값을 활용해 각각 Action에 따른 [s, d, s_dot, d_dot] 값의 평균/분산을 구하는 알고리즘입니다.



      def train(self, X, Y):
        '''
        Collect the data and calculate mean and standard variation
        for each class. Record them for later use in prediction.
        '''
        # TODO: implement code.
        Train = {}
        for label in self.classes:
            Train[label] = []
        for label in Train:
            for i in range(len(X)):
                if label == Y[i]:
                    Train[label].append(X[i])
                    
        self.Statistics = {label :{} for label in self.classes} 
        for label in self.Statistics:
            self.Statistics[label]['mean'] = np.mean(Train[label],axis = 0)
            self.Statistics[label]['std'] = np.std(Train[label],axis = 0)
            
        return self.Statistics 

 - for 문을 이용해 데이터를 [좌회전 / 차선유지 / 우회전]에 따라 각각 분류 하였습니다.
 - 분류된 각 데이터에 대한 평균과 분산을 구하도록 하였습니다.



(2) Prediction Algorithm

 - 학습된 데이터로부터 구한 평균/ 분산값을 이용해 Input Data가 어떠한 Action을 취하는지 찾아내는 알고리즘입니다.

    # Given an observation (s, s_dot, d, d_dot), predict which behaviour
    # the vehicle is going to take using GNB.
    
       def predict(self, observation):
        '''
        Calculate Gaussian probability for each variable based on the
        mean and standard deviation calculated in the training process.
        Multiply all the probabilities for variables, and then
        normalize them to get conditional probabilities.
        Return the label for the highest conditional probability.
        '''
        # TODO: implement code.
        probs = {}
        for label in self.classes:
            prob = 1
            for i in range(len(observation)):
                prob *= gaussian_prob(observation[i], self.Statistics[label]['mean'][i], self.Statistics[label]['std'][i])
            probs[label] = prob
        
        prediction = max(probs,key=probs.get)
        
        

        return prediction

 - 학습 데이터로 부터 얻은 [s, d, s_dot, d_dot]의 평균값과 분산에 따른 각 Action에 대한 Gaussian 확률 분포를 계산하여 가장 확률이 큰 값을 Prediction Action으로 리턴한다.






## Assignment 2 - Behavior Planning

- 과제 목표
    Behaviour Planning은 설정된 Cost Function(목적지까지의 종/횡 거리, 차선별 속도)에 따라 차량 거동에 있어 최적의 Trajectory를 찾아가는 알고리즘입니다.


(1) Choose Next State

 - choose_next_state 함수는 현재 차량의 상태에서 다음 state로 넘어갈 수 있는 모든 Trajectory를 고려할때 가장 cost가 작은 궤적을 찾는 함수입니다.


    def choose_next_state(self, predictions):
        '''
        Implement the transition function code for the vehicle's
        behaviour planning finite state machine, which operates based on
        the cost function (defined in a separate module cost_functions.py).

        INPUTS: A predictions dictionary with vehicle id keys and predicted
            vehicle trajectories as values. Trajectories are a list of
            Vehicle objects representing the vehicle at the current timestep
            and one timestep in the future.
        OUTPUT: The the best (lowest cost) trajectory corresponding to
            the next ego vehicle state.

        Functions that will be useful:
        1. successor_states():
            Returns a vector of possible successor states
            for the finite state machine.

        2. generate_trajectory(self, state, predictions):
            Returns a vector of Vehicle objects representing a
            vehicle trajectory, given a state and predictions.
            Note that trajectories might be empty if no possible trajectory
            exists for the state; for example, if the state is LCR, but a
            vehicle is occupying the space to the ego vehicle's right,
            then there is no possible trajectory without first
            transitioning to another state.

        3. calculate_cost(vehicle, trajectory, predictions):
            Imported from cost_functions.py, computes the cost for
            a trajectory.
        '''

        # TODO: implement state transition function based on the cost
        #       associated with each transition.

        # Note that the return value is a trajectory, where a trajectory
        # is a list of Vehicle objects with two elements.
        possible_successor_states = self.successor_states()
        costs = []
        
        for state in possible_successor_states :
            trajectory = self.generate_trajectory(state, predictions)
            cost = calculate_cost(self, trajectory, predictions)
            costs.append({'state': state, 'cost' : cost, 'trajectory' : trajectory})
        
        best_next_state = None
        min_cost = 9999999
        
        for x in costs:
            if x['cost'] < min_cost:
                min_cost = x['cost']
                best_next_state = x['state']
                best_next_trajectory = x['trajectory']
        

        return best_next_trajectory

 - 현재 차량 위치에서 천이 가능한 다음 state들은 successor_state 함수로 부터 받아 올 수 있습니다.
 - 이후 다음 state로 천이 할때의 Trajectory를 generate_trajectory함수로 생성하고 이 Trajectory에 대한 Cost를 미리 지정된 calculate_cost 함수를 사용하여 계산합니다.
 - 각각의 Trajectory에 대해서 계산된 cost가 가장 작은 Trajectory를 찾아내고 이 Trajectory를 best_next_Trajectory로 지정하여 리턴합니다.



(2) Cost Function

 - Cost Fuction은 goal까지의 종/횡 방향 거리에 대한 Cost Function 그리고 지정된 차선에서의 제한된 속도에 따른 Cost Function 두가지를 사용하여 정의 됩니다.

 1) goal_distance_cost


   def goal_distance_cost(vehicle, trajectory, predictions, data):
    '''
    Cost increases based on distance of intended lane (for planning a
    lane change) and final lane of a trajectory.
    Cost of being out of goal lane also becomes larger as vehicle approaches
    the goal distance.
    '''
    lat_offset = abs(2*vehicle.goal_lane - data.intended_lane - data.final_lane)
    distance = abs(data.end_distance_to_goal)
    
    if distance:
        cost = 1-exp(-lat_offset/ distance)
    else:
        cost = 1
    
    return cost

    - goal_distance_cost는 차량의 현재 차선이 Goal의 차선과 멀수록 그리고 Goal과의 거리가 가까울 수록 큰 값의 Cost를 리턴하는 함수이며 Cost Function은 exponential 함수로 나타내었습니다.
    - Cost Function에 따라 distance 값이 가까워 질 수록 Cost를 크게 생성하여 Cost를 작은 방향으로 만들어 주기 위해 Goal_Lane으로 차선을 변경하게 해주도록 하였습니다.

 2) inefficiency_cost

   def inefficiency_cost(vehicle, trajectory, predictions, data):
    '''
    Cost becomes higher for trajectories with intended lane and final lane
    that have slower traffic.
    '''
    intended_speed = velocity(predictions, data.intended_lane) or vehicle.target_speed
    final_speed =  velocity(predictions, data.final_lane) or vehicle.target_speed
    
    cost = float((2*vehicle.target_speed - intended_speed - final_speed)/vehicle.target_speed)
    
    
    return cost

    - inefficiency cost 함수는 차량이 현재 위치한 Lane의 속도가 느릴 수록 큰 Cost를 리턴하는 함수입니다.
    - 도착지까지 거리가 멀 경우 가장 빠른 속도를 유지할 수 있는 차선에서 주행을 하도록 하였습니다.
    - Cost Function은 차량의 최고 속도와 현재 속도 사이의 선형함수를 사용하여 Cost를 작은 방향으로 만들어 주기 위해 최고 속도와 현재 속도의 차가 작은 차선으로 변경 할 수 있도록 하였습니다.
