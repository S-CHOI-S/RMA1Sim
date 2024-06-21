# Developing an Adaptive Locomotion Strategy for a Quadruped Robot
> [MEU5053] Machine Learning And Programming (2024Y: First Semester)

<br/>

## Installation
__Essential 1.__ Dependencies
> Install the following packages before using RaiSim: eigen library, cmake
```shell
sudo apt install libeigen3-dev cmake
```

__Essential 2.__ RaiSim
> RaiSim is the physics engine developed by RaiSim Tech Inc.: [raisimTech/raisimLib](https://github.com/raisimTech/raisimLib.git)
```shell
git clone https://github.com/S-CHOI-S/RaiSim-RL.git
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
cd {$workspace_dir}/raisimLib/raisimGymTorch
```
```shell
python setup.py develop
```
