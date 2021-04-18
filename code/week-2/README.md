# Week 2 - Markov Localization

---

[//]: # (Image References)
[plot]: ./markov.gif

## Assignment

You will complete the implementation of a simple Markov localizer by writing the following two functions in `markov_localizer.py`:

* `motion_model()`: For each possible prior positions, calculate the probability that the vehicle will move to the position specified by `position` given as input.
* `observation_model()`: Given the `observations`, calculate the probability of this measurement being observed using `pseudo_ranges`.

The algorithm is presented and explained in class.

All the other source files (`main.py` and `helper.py`) should be left as they are.

If you correctly implement the above functions, you expect to see a plot similar to the following:

![Expected Result of Markov Localization][plot]

If you run the program (`main.py`) without any modification to the code, it will generate only the frame of the above plot because all probabilities returned by `motion_model()` are zero by default.

---

# HW_01 Markov Localizaion

## Assignment	

**(1) Motion Model


    def motion_model(position, mov, priors, map_size, stdev):
        # Initialize the position's probability to zero.
        position_prob = 0.0
        
        for i in range(map_size):
            position_prob += norm_pdf(position-i, mov ,stdev)*priors[i]

Motion Model은 차량이 이전 위치에서 Control Input으로 인해 움직인 현재 위치에 대한 확률을 나타냅니다.
- Input Data
 * position : position은 0 ~ 25로 표현되는 전체 Map의 각각의 위치를 나타냅니다.
 * mov : 매 timestep마다 차량을 이동을 시키는 Control Input으로 1.0으로 설정되어있습니다.
 * stdev : Control Noise로서 Control Input의 불확실성을 나타냅니다.
 * priors[i] : 이전 Timestep에서 차량이 가질수 있는 Map의 각각 위치에의 확률을 나타냅니다.

- Position Proability는 차량이 존재 할 수 있는 모든 위치에서의 확률 값을 가져와 이전 위치에서 현재 위치로 이동할 확률을 계산합니다.
- Map의 각각의 위치에서 현재 위치까지 도달할 수 있는 확률을 계산한 뒤 합계를 구하면 현재 위치의 존재할 확률을 알 수 있다.

- 특정 위치에서 현재 위치로 도달 할 확률은 (차량이 이전 위치에서 Control Input에 의해 현재 위치로 이동할 확률)*(차량이 이전 위치에 존재할 확률)로 계산할 수 있다.

        

**(2) Observer Model


    # Observation model (assuming independent Gaussian)
    def observation_model(landmarks, observations, pseudo_ranges, stdev):
        # Initialize the measurement's probability to one.
        distance_prob = 1.
        
        if len(observations) == len(pseudo_ranges):
            for i in range(len(pseudo_ranges)):
                distance_prob *= norm_pdf(observations[i],pseudo_ranges[i],1)
        else: 
            distance_prob = 0
        return distance_prob


Observer Model은 현재 위치에서 측정된 Landmark가 존재 할 확률을 나타낸다.
- Input Data
 * landmarks : Map에서 나타나는 Landmark의 고정된 위치를 나타낸다.
 * observations : 현재 timestep에서 측정되는 Landmark와의 거리 값을 List로 받아온다.
 * pseudo_ranges : pseudo_ranges는 Map으로부터 받아오는 각각의 위치에서 전방에 보이는 Landmark들과의 거리를 나타내며 후방에 있는 Landmark들은 관측하지 않는다.	
 * stdev : observation에 대한 분산으로 Sensor의 Noise에 의한 불확실성을 나타내며 1.0으로 설정되어있다.


- Sensor에 의해 관측된 Landmark의 갯수가 Map에 의해 받아온 전방의 Landmark의 수(Pseuo_ranges)가 다르면 잘못된 값으로 판단하여 확률은 0으로 계산한다.
- 두 데이터의 수가 같다면 Map의 각각 위치에서 측정값의 위치에 대한 확률을 계산할 수 있다.
- 각 확률은 현재 위치에서 Map에 의해 얻은 Landmark의 위치에서 측정값으로 부터 얻은 위치가 일치할 확률을 구한다.
- 각각의 Landmark에 대한 확률을 모두 고려해야하기 때문에 각 Landmark가 존재할 확률을 모두 곱하여 계산한다.



