# Developing an Adaptive Locomotion Strategy for a Quadruped Robot
#### [MEU5053] Machine Learning And Programming (2024Y: First Semester)  
> This project referred to Antonilo's research on [rl_locomotion](https://github.com/antonilo/rl_locomotion)

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

### 2. RMA Algorithm
> __[Rapid Motor Adaptation (RMA)](https://arxiv.org/abs/2107.04034)__ algorithm can solve the problem of real-time adaptation to unseen scenarios like changing terrains, changing payload, wear and tear for successful real-world deployments.
<div style="float: left;">
  <img src="https://github.com/S-CHOI-S/RaiSim-RL/assets/113012648/e1766812-8d26-4bd2-9f88-6963267f685f" width="70%" height="70%"/>
</div>


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

