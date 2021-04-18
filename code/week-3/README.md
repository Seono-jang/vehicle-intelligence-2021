
# HW_02 Extended Kalman Filter

## Assignment

- 과제 목표
  이 과제의 목표는 Radar와 Lidar로 부터 얻은 정보를 바탕으로 임의의 Dynamic Object의 위치를 추정하고 Radar와 Lidar 각각의 정보를 Sensor Fusion 하여 Object들의 정확한 위치를 추정해 내는 것입니다.
  Lidar는 물체의 위치를 (x,y)좌표로 나타낼 수 있는 반면 Radar는 상대 거리와 상대 각도만을 알 수 있으므로 시스템 모델이 다르게 표현됩니다.
  Radar의 경우 상대 거리와 상대 각도를 (x,y)에 대하여 비선형성을 나타내므로 Jacobian 행렬식을 구하여 선형화를 해주어야합니다.

- Update EKF
 Update_EKF는 Radar로부터 관측된 Object들의 상대 거리와 각도로 부터 Extended Kalman Filter를 사용해 물체의 (x,y) 위치를 추정하는 함수입니다.
	
    def update_ekf(self, z):
        # TODO: Implement EKF update for radar measurements
        # 1. Compute Jacobian Matrix H_j
        # 2. Calculate S = H_j * P' * H_j^T + R
        # 3. Calculate Kalman gain K = H_j * P' * Hj^T + R
        # 4. Estimate y = z - h(x')
        # 5. Normalize phi so that it is between -PI and +PI
        # 6. Calculate new estimates
        #    x = x' + K * y
        #    P = (I - K * H_j) * P
        if z[1] < 0 :
            z[1] = z[1] + 2*np.pi
        
        self.H_j = Jacobian(self.x)
        
        S = np.dot(np.dot(self.H_j, self.P), self.H_j.T) + self.R
        K = np.dot(np.dot(self.P, self.H_j.T), np.linalg.inv(S))
        y = z - output_matrix(self.x)
        print(output_matrix(self.x))
        #print(self.x)
        self.x = self.x + np.dot(K,y)
        
        self.P = self.P - np.dot(np.dot(K, self.H_j), self.P)

각 timestep에서 Object의위치값과 오차 공분산의 예측값을 받아와 Extended Kalman Filter로 추정값을 계산해내는 부분입니다.

먼저 Radar 데이터를 Jacobian 행렬식을 활용해 선형화를 해주었습니다.
Jacobian 행렬식에 대한 구현은 tools.py에 구현하였으며 구현식은 다음과 같습니다.	

    def output_matrix(x):
        px, py, vx, vy = x
        if px == 0 and py == 0:
            print("Error: both px and py are zero while trying to")
            print("       calculate the output matrix.")
            return np.zeros(3)
        H_x = np.array([
            sqrt(px*px + py*py),
            atan2(py,px),
            (px * vx + py * vy) / sqrt(px * px + py * py)
        ])
         
        if H_x[1] < 0 :
            H_x[1] = H_x[1] + 2*np.pi
             
        return H_x

측정 값에서 받아오는 상대각도가 180도를 넘어가는 시점에서 (-)로 전환되는 문제가 있어 상대각도가 (-)부호가 될때 2pi 만큼 더해주었습니다.

이 후 각각 timestep마다 받아오는 측정값과 예측값의 오차를 Kalman Filter를 활용하여 Object의 위치값을 추정할 수 있습니다.
