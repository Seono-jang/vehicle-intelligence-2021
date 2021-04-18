
# HW_03 Particle Filter

## Assignment

- 과제 목표
    Particle Filter를 이용한 Localization 알고리즘은 무작위로 선정한 다수의 파티클을 이용하여 확률론적에 근거하여 자차의 위치를 추정해내는 알고리즘입니다.
    Particle Filter는 다수의 Particle을 활용해 비선형성 모델의 확률분포를 예측하고, 측정값으로부터 파티클들의 가중치를 보정하여 timestep에 따라자차의 위치를 찾아내 갈 수 있습니다.

(1) Update_weights

 - Update_weights 함수는 각 Particle의 위치에서 Landmark까지의 좌표를 측정한 데이터와 Map상의 데이터값을 비교하여 각 Particle이 현재 내 차량의 위치일 확률을 계산하는 함수입니다.
 - Update_weights 함수는 총 5단계로 구성 되어있습니다.


       def update_weights(self, sensor_range, std_landmark_x, std_landmark_y,
                       observations, map_landmarks):

        for p in self.particles:

1_step - 현재 particle의 위치에서 Sensor로 관측가능한 Landmark들으 분류하는 과정입니다.

            visible_landmark = []
            for landmark_id in map_landmarks:
                dist = distance(map_landmarks[landmark_id],p)
                if dist <= sensor_range:
                    #visible_landmark.append({landmark_id : map_landmarks[landmark_id]})
                    visible_landmark.append({'id' :landmark_id , 'x' : map_landmarks[landmark_id]['x'], 'y' : map_landmarks[landmark_id]['y']})

2_step - 각 Particle의 Local 좌표계에서 측정된 Landmark의 위치를 Global 좌표계로 변환하여 Map 데이터와 동일한 좌표계로 만듭니다.


            #좌표변환
            
            coordinate_transform = []
            for o in observations:
                #A = np.array([[np.cos(p['t']), -np.sin(p['t']), p['x']],[np.sin(p['t']), np.cos(p['t']), p['y']],[0, 0, 1]])
                #B = np.array([o['x']], [o['y']], 1)

                x = np.cos(p['t']) * o['x'] - np.sin(p['t']) * o['y'] + p['x']
                y = np.sin(p['t']) * o['x'] + np.cos(p['t']) * o['y'] + p['y']
                coordinate_transform.append({'x': x, 'y': y})

            
3_step - Particle에서 측정된 Object들이 Map상에서 어떤 Landmark와 가장 가까운지 판단하고 Observations의 리스트와 동일한 index에서 비교대상인 Landmark의 ID와 (x,y)좌표의 List를 형성합니다.
         
         
         #3 Associate each transformed observation to one of the predicted landmark positions
            #p['assoc'] 
            if not visible_landmark:
                continue
            associations = self.associate(visible_landmark,coordinate_transform)

4_step - 연관된 Map상의 Landmark에서 좌표 변환된 측정값이 존재할 확률을 계산하여 각 Particle이 차량의 실제 위치일 확률을 계산합니다.
           확률 계산은 2차 가우시안 분포 함수를 이용하여 계산하였고 현재 Particle에서 관측할 수 있는 Landmark의 ID를 List에 저장하였습니다.
         
            #num = 0
            p_association = []
            for i in range(len(associations)):
                
                one_over_sqrt_2pi = 1 / (2 * np.pi*std_landmark_x*std_landmark_y)
                
                z = ((coordinate_transform[i]['x'] - associations[i]['x'])**2 / std_landmark_x**2 + (coordinate_transform[i]['y'] - associations[i]['y'])**2 / std_landmark_y**2)
                weight = (one_over_sqrt_2pi) * exp(-0.5*z) + 1e-10
                p_association.append(associations[i]['id'])

            p['w'] = weight
            p['assoc'] = p_association




(2) Resampling

- Resampling 함수는 이전 단계에서 계산된 각 Particle들의 Weight에 따라 Weight의 크기에 따라 Particle의 구성을 재 샘플링하는 과정입니다.
- 이전에 구해진 weight들을 Normalize 하여 numpy.random.choice 함수를 이용해 Particle의 분포를 weight의 크기에 따라 재 구성하여 새로운 Particle들의 List를 만들었습니다.

        def resample(self):
        
        new_particles = []
        weights = []
        for p in self.particles:
            weights.append(p['w'])
            
        weights /= np.sum(weights)
        
        idx = np.random.choice(self.num_particles,self.num_particles,p=weights)
        
        for i in idx:
            new_particles.append(copy.deepcopy(self.particles[i]))
        
        self.particles = new_particles
