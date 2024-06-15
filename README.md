# Developing an Adaptive Locomotion Strategy for a Quadruped Robot
#### [MEU5053] Machine Learning And Programming (2024Y: First Semester)  
> This project referred to Antonilo's research on [rl_locomotion](https://github.com/antonilo/rl_locomotion)

- __Goal of the project:__  
  Our goal is to train a __quadruped robot__ using __reinforcement learning (RL)__. The objective is to enable the robot to learn robust walking and running on various surfaces, including flat terrain and rough terrain. Quadruped robots offer greater stability and speed compared to bipedal robots, making them suitable for applications such as military operations, factory management, and other fields. By leveraging reinforcement learning, we aim to enhance the capabilities of quadruped robots, making them more adaptable and versatile in various fields. Reinforcement learning allows the robot to learn from its experiences, improving its performance over time as it encounters different scenarios and terrains. Through iterative training and real-world testing, we hope to create quadruped robots that are not only capable but also resilient and reliable in performing their designated tasks.

- __The importance of training on rough terrain:__  
  Training quadruped robots on rough terrain is crucial as it significantly enhances the robots' stability, adaptability, practicality, durability, obstacle-handling capabilities, and problem-solving skills. The real world is not always flat, and for quadruped robots to operate effectively in various environments, adaptability to rough terrain is essential. This training enables the robots to maintain balance and navigate unpredictable obstacles or irregular surfaces stably, maximizing performance in diverse real-world applications such as military operations, disaster response, and exploration. Additionally, training on rough terrain makes robots more robust and durable, reducing maintenance costs and downtime, and allows them to efficiently recognize and overcome various obstacles, increasing mission success rates and ensuring safe operation. Through reinforcement learning, robots experience diverse scenarios, improving problem-solving abilities and enabling better path planning and decision-making.

- __Project Badges:__
  
  <img src="https://github.com/S-CHOI-S/RaiSim-RL/assets/113012648/5cf57e38-41c6-4639-9de8-b93a78283984" width="3.5%"/> ![Python](https://img.shields.io/badge/python-3670A0?style=for-the-badge&logo=python&logoColor=ffdd54)
![C++](https://img.shields.io/badge/c++-%2300599C.svg?style=for-the-badge&logo=c%2B%2B&logoColor=white)
![PyTorch](https://img.shields.io/badge/PyTorch-%23EE4C2C.svg?style=for-the-badge&logo=PyTorch&logoColor=white)
![ROS](https://img.shields.io/badge/ROS-22314E?style=for-the-badge&logo=ROS&logoColor=white)
![Unity](https://img.shields.io/badge/unity-%23000000.svg?style=for-the-badge&logo=unity&logoColor=white)

  <img src="https://github.com/S-CHOI-S/RMA1Sim/assets/113012648/2ea45869-1ed0-47af-a532-2da49b58f787" width="70%"/>

<br/>

## Contributions
### 1. PPO Algorithm
> __[Proximal Policy Optimization (PPO)](https://en.wikipedia.org/wiki/Proximal_policy_optimization)__ is a type of reinforcement learning and is the best-performing algorithm for learning the walking of quadrupedal robots.

* __Simplicity__  
  Compared with the TRPO, the PPO method is relatively easy to implement and takes less computation time.  
  Therefore, it is cheaper and more efficient to use PPO in large-scale problems.
* __Stability__  
  While other reinforcement learning algorithms require hyperparameter tuning, PPO does not necessarily require hyperparameter tuning.  
  Also, PPO does not require sophisticated optimization techniques.  
  It can be easily practiced with standard deep learning frameworks and generalized to a broad range of tasks.
* __Sample efficiency__  
  PPO achieved sample efficiency because of its usage of surrogate objectives.  
  The surrogate objectives enable PPO to avoid the new policy changing too far from the old policy;  
  the clip function regularizes the policy update and reuses training data.  

  ✔️ _Probability Ratio_
    $$\rho(s,a) = {{π_θ(a_t∣s_t)} \over {π_{θ_{old}}(a_t∣s_t)}}$$
  
  ✔️ _Objective Function_
    $$\max_θ \mathbb{E}[min(ρ(s,a)⋅A(s,a),clip(ρ(s,a),1−ϵ,1+ϵ)⋅A(s,a))]$$

### 2. RMA Algorithm
> __[Rapid Motor Adaptation (RMA)](https://arxiv.org/abs/2107.04034)__ algorithm can solve the problem of real-time adaptation to unseen scenarios like changing terrains, changing payload, wear and tear for successful real-world deployments.

- __Structure of RMA__  
  RMA consists of a base policy and an adaptation module, each performing specific roles during training and deployment. The base policy is trained in simulation using model-free RL, while the adaptation module predicts environmental factors from real-world data. During deployment, the adaptation module generates environmental factors at a lower frequency, and the base policy uses these factors at a higher frequency to determine the robot's actions. This asynchronous design enables smooth operation on low-cost robots with limited computational resources.
  - __Training in Simulation__  
  In the first phase, the base policy $\pi$ takes as input the current state $x_t$, previous action $a_{t-1}$, and the privileged environmental factors $e_t$. These environmental factors are encoded into the latent extrinsics vector $z_t$ using the environmental factor encoder $\mu$. The base policy is trained in simulation using model-free reinforcement learning (RL). In the second phase, the adaptation module $\phi$ is trained via supervised learning to predict the extrinsics $\hat{z}_t$ from the history of states and actions.
  - __Deployment__  
    During deployment, the adaptation module $\phi$ generates the extrinsics $\hat{z}_t$ at 10Hz, and the base policy generates the desired joint positions at 100Hz, which are converted to torques using A1's PD controller. Since the adaptation module operates at a lower frequency, the base policy uses the most recent extrinsics vector $\hat{z}_t$ predicted by the adaptation module to predict $a_t$.

  &nbsp;&nbsp;&nbsp;<img src="https://github.com/S-CHOI-S/RMA1Sim/assets/113012648/af132998-039a-4230-87b3-b1a508752da5" width="80%" height="80%"/>

### 3. Diversity is All You Need
> __[Diversity is All You Need (DIAYN)](https://arxiv.org/abs/1802.06070)__ is an algorithm focused on generating diverse behaviors in reinforcement learning, promoting variety in control tasks to help agents learn more generalized actions. Its primary goal is to train agents on diverse policies, enabling them to adapt to various situations.

- __Limitations of RMA__
  - __Lack of diverse behavior learning__  
    RMA focuses primarily on achieving specific goals, which can limit its ability to learn diverse behavior patterns. This may restrict adaptability and flexibility in new situations.
  - __Difficulty in using intrinsic rewards__  
    RMA tends to rely on external rewards, making effective learning challenging in situations with sparse reward signals. This means that significant effort is required in reward engineering.
- __Application of DIAYN__  
  - __Diverse Behavior Learning__  
    DIAYN encourages the agent to learn a variety of skills (behaviors). This allows the agent to adapt to different situations by learning diverse behavior patterns, making it better able to respond to changes in the environment.
  - __Intrinsic Reward Structure__  
    DIAYN learns using intrinsic reward functions without relying on external rewards. This reduces the need for complex reward engineering and enables the agent to effectively learn in various situations.
  - __Generalized Policy Learning__  
    DIAYN trains the agent on diverse policies, enabling it to perform generalized behaviors across multiple environments. This helps overcome the adaptation limitations of RMA.
  - __Enhanced Exploration__  
    DIAYN encourages the agent to try diverse behaviors, thereby enhancing exploration. This increases adaptability in new environments and ensures efficient performance in a variety of situations.

  <img src="https://github.com/S-CHOI-S/RMA1Sim/assets/113012648/011c342b-8068-411e-b8d7-982d06a28793" width="80%"/>

### 4. Jumping in Discontinuous Terrain
> __Jumping in discontinuous terrain__ significantly enhances the mobility and adaptability of quadruped robots, enabling efficient and safe operation in various missions.

- **Obstacle Navigation and Speed Enhancement**  
   Jumping allows the robot to quickly overcome obstacles such as rocks, branches, and gaps, enabling it to traverse complex terrain more efficiently and faster than walking.
- **Safe and Adaptable Movement**  
   Jumping ensures safe navigation over small gaps or cracks in discontinuous terrain and enhances the robot's adaptability to various environments, expanding its usability.
- **Improved Mission Capabilities**  
   Equipped with jumping abilities, the robot can perform efficiently in diverse missions like exploration, rescue, and military operations.

<a href="https://www.mdpi.com/2313-7673/8/1/36"><img src="https://www.mdpi.com/biomimetics/biomimetics-08-00036/article_deploy/html/images/biomimetics-08-00036-g001.png"/></a>

### 5. Sim-to-Real
> __Sim-to-real__ refers to the ability of a model trained in a simulated environment to perform effectively in real-world conditions. It's commonly used in robotics and artificial intelligence, ensuring seamless transition from simulation to reality.

- __Real-world Performance Evaluation__  
  Simulations may not perfectly replicate the real world and unexpected issues can arise. Therefore, it is important to evaluate and test algorithms on actual robots.
- __Adaptation to Environmental Changes__  
  Simulations may struggle to anticipate unexpected environmental changes or external factors. Using real robots allows for real-time detection and adaptation to environmental conditions.
- __Assessment of Robot's Actual Behavior__  
  Simulations often make assumptions about ideal behavior or constraints that may not hold true for real robots. Thus, evaluating algorithms on real robots provides feedback on their actual performance.
- __Realistic Data Collection__  
  Data collected by robots in real environments is more realistic and useful than data generated in simulations. This data aids in developing more accurate modeling and effective control algorithms.
- __Practicality__  
  Robot technology must be applied in the real world to be useful. Therefore, testing and developing algorithms on real robots is essential.

&nbsp;&nbsp;&nbsp;<img src="https://github.com/S-CHOI-S/RMA1Sim/assets/113012648/bce3c4e1-32a5-4d7d-a969-d80384b5756e" width="70%"/>

<br/>

## Installation
__Essential 1.__ Dependencies
> Install the following packages before using RaiSim: eigen library, cmake
```shell
$ sudo apt install libeigen3-dev cmake
```

__Essential 2.__ RaiSim
> RaiSim is the physics engine developed by RaiSim Tech Inc.: [raisimTech/raisimLib](https://github.com/raisimTech/raisimLib.git)
```shell
$ git clone https://github.com/S-CHOI-S/RaiSim-RL.git
```
> [!note]
> After cloned the RaiSim repository, you should follow the instructions on [installation page](https://raisim.com/sections/Installation.html)

> [!Caution]
> Do not miss the activation key!

<div style="float: left;">
  <img src="https://github.com/S-CHOI-S/RaiSim-RL/assets/113012648/7bb72ffe-b6fe-473c-9d71-177cc363065e" width="60%" height="60%"/>
</div>
</br>

__Essential 3.__ RaisimGymTorch
> RaisimGymTorch is a gym environment example with Raisim.  
> A simple pytorch-based RL framework is provided as well but it should work well with any other RL frameworks!

```shell
$ cd {$workspace_dir}/raisimLib/raisimGymTorch
```
```shell
$ python setup.py develop
```

<br/>

## Usage
__Step 1.__ Train the policy
> GPU_NUM can be found with the command ```nvidia-smi```!  
> EXPT_ID will be your data file number.
```shell
$ cd {$workspace_dir}/raisimLib/raisimGymTorch/raisimGymTorch/env/envs/rsg_a1_task
```
```shell
$ python runner.py --name random --gpu GPU_NUM --exptid EXPT_ID 
```

__Step 2.__ Run RaisimUnity
> You can check the robot using the Unity renderer.
```shell
$ cd {$workspace_dir}/raisimLib/raisimUnity/"OS" # e.g. cd raisim_ws/raisimLib/raisimUnity/linux
```
``` shell
$ ./raisimUnity.x86_64
```

__Step 3.__ Visualize the policy
> You may run the code in your conda env!
```shell
$ cd {$workspace_dir}/raisimLib/raisimGymTorch/raisimGymTorch/env/envs/rsg_a1_task
```
> EXPT_ID and POLICY_ID can be found in the ```raisimGymTorch/data/rsg_a1_task``` directory!
```shell
$ python viz_policy.py ../../../../data/rsg_a1_task/EXPT_ID POLICY_ID
```
<img align="left" src="https://github.com/S-CHOI-S/RaiSim-RL/assets/113012648/955bd5e6-28eb-443d-8f11-0275cb1e2773" width="45%"/>
</br></br></br>

```shell
# e.g. EXPT_ID = 0001, POLICY_ID = 2000
$ python viz_policy.py \
  ../../../../data/rsg_a1_task/0001 2000
```
</br></br>

> [!Tip]
> If you want to download the pretrained data, please download the [RMA1Sim](https://drive.google.com/drive/folders/1mAwB3S2jnIrxw_yQOmWcerQ_lsq68K87?usp=sharing) file!

Additionally, the data files should be placed as follows:
```shell
raisimGymTorch
├── build
├── CMakeLists.txt
├── data
│   └── anymal_locomotion
│       └── jump
├── raisimGymTorch
├── ...
```

```shell
raisimGymTorch
├── build
├── CMakeLists.txt
├── data
│   └── anymal_locomotion
│       └── anymal_mod3
├── raisimGymTorch
├── ...
```

</br>

## Results
### 1. RMA (Rapid Motor Adaptation)
> [!Note]
> You can directly execute the results using the ['RMA' tag](https://github.com/S-CHOI-S/RMA1Sim/tree/RMA).

- __Random ENV1:__ Rough Terrain  
- __Random ENV2:__ Stairs

  <img src="https://github.com/S-CHOI-S/RMA1Sim/assets/113012648/6851358e-9434-492f-b106-bcacf3fcda13" width="60%"/>

### 2. Multi-directional policy
> [!Note]
> You can directly execute the results using the ['multi_direction' tag](https://github.com/S-CHOI-S/RMA1Sim/tree/multi_direction).

> Red line means __target trajectory__ and blue line means __robot trajectory__.

- __The graph below shows the target direction values from left to right as __-π/2, -π/4, 0, π/4, and π/2__, respectively.__

  <img src="https://github.com/S-CHOI-S/RMA1Sim/assets/113012648/63db6a47-4f31-43a5-b2f5-c95e3c5377f6"/>
  ❗ The error value increases as the robot moves.

- __Enables the robot to change movements based on user commands.__
  
  (Experiment with half-circle movements to test direction changes).

  <div style="display: flex; justify-content: space-around;">
    <img src="https://github.com/S-CHOI-S/RMA1Sim/assets/113012648/46b41040-4e79-4517-90ef-6e6d1fe10613" width="45%" />
    <img src="https://github.com/S-CHOI-S/RMA1Sim/assets/113012648/17074475-1864-4c3e-aed7-d6c4b24af527" width="46%" />
  </div>
  ❗ The graph shows the significant errors. To reduce the steady state error, the robot needs to know its own movements!

- __Improvements__  
  Additional observation of the angle difference between the current direction and the intended direction for better learning.

  <img src="https://github.com/S-CHOI-S/RMA1Sim/assets/113012648/d3431ec4-bd6b-47c5-86de-3bdc0fe25e53" width="90%"/>
  
  ❗ Reduction in errors compared to previous methods observed!

  ❗ __Application:__ Robot moves to avoid cylindrical shapes, demonstrating practical use of learned policies.




## Discussion

## Feedback Q&A
- __How did you modulate the reward function, particularly after jumping to landing?__  
  To design the motion for jumping to the desired location, we divided the process into three stages: the takeoff phase, the aerial phase, and the landing phase, as different actions are required for each state.
  - __Takeoff Phase:__  
    The primary goal is to execute the jump rather than to move closer to the target location. Therefore, we designed a reward function specifically for the jumping action.
    
    ✔️ _Jump Velocity_  
    $$reward_{jump}\ +=\ exp(-5(v_y-v_{y, ref})^2+(v_z-v_{z,ref})^2)$$
    
    ✔️ _Keep Body Direction_
    $$reward_{jump}\ +=\ exp \left(-4 \left|\frac {\overrightarrow v_{global}}{\left|\overrightarrow v_{global} \right|} - \frac {\overrightarrow v_{local}}{\left|\overrightarrow v_{local} \right|} \right|\right)$$
    
    ✔️ _Keep Body Twisting_
    $$reward_{jump}\ +=\ exp(-|w_x|-10|w_z|)$$
    
  - __Aerial Phase:__  
    During the aerial phase, we added a reward component based on the distance to the target location. This adjustment helps the system to refine the jump direction towards the target as learning progresses.
    
    ✔️ _Jump Direction_  
    $$reward_{position}\ +=\ exp(-3 \sqrt{(x-x_{goal})^2+(y-y_{goal})^2}\ )$$
    
  - __Landing Phase:__  
    Finally, in the landing phase, we introduced a reward function that includes terms for limiting joint movements and returning to previous joint angles. This approach helps the system learn stable landing motions.
    
    ✔️ _Joint Movement & Stable Landing_  
    $$reward_{landing}\ +=\ exp \left(-3 \left( \sqrt{\sum w_{joint}^2} + \sqrt{\sum (\theta_{joint} - \theta_{joint_{init}})^2 }\ \right) \right)$$

- __Why do you think errors compounded in the Direction Changing Policy?__
- __What methods did you use to input the control commands?__
- __An integral action is needed in the reward function to eliminate offset errors.__


### Reference
- https://raisim.com/
- https://github.com/raisimTech/raisimLib.git
- https://arxiv.org/abs/1707.06347
- https://arxiv.org/abs/2107.04034
- https://github.com/antonilo/rl_locomotion.git
- https://ashish-kmr.github.io/rma-legged-robots/
  
