## raisim_env_anymal

### How to use this repo
There is nothing to compile here. This only provides a header file for an environment definition. Read the instruction of raisimGym. 

### Dependencies
- raisimgym

### Run
1. Compile raisimgym: ```python setup.py develop```
2. run runner.py of the task (for anymal example): ```cd raisimGymTorch/env/envs/rsg_anymal && python raisimGymTorch/env/envs/rsg_anymal/runner.py```

### Test policy
1. Compile raisimgym: ```python setup develop```
2. run tester.py of the task with policy (for anymal example): ``` python raisimGymTorch/env/envs/rsg_anymal/tester.py --weight data/anymal_locomotion/2024-05-22-10-51-35/full_3000.pt ```
python raisimGymTorch/env/envs/rsg_anymal/tester.py --weight data/anymal_locomotion/test_rotate/full_5600.pt
python raisimGymTorch/env/envs/rsg_anymal/tester.py --weight data/anymal_locomotion/test_jump/full_5000.pt
python raisimGymTorch/env/envs/rsg_anymal/tester.py --weight data/anymal_locomotion/test_final/full_3000.pt
2024-06-03-13-07-52
python raisimGymTorch/env/envs/rsg_anymal/tester.py --weight data/anymal_locomotion/ani/full_3000.pt


### Retrain policy
1. run runner.py of the task with policy (for anymal example): ``` python raisimGymTorch/env/envs/rsg_anymal/runner.py --mode retrain --weight data/anymal_locomotiontest_jump/full_5000.pt```

### Debugging
1. Compile raisimgym with debug symbols: ```python setup develop --Debug```. This compiles <YOUR_APP_NAME>_debug_app
2. Run it with Valgrind. I strongly recommend using Clion for 
s
[text](<data/anymal_locomotion/test2_before _Height>)
python raisimGymTorch/env/envs/rsg_anymal/runner.py --mode retrain --weight data/anymal_locomotion/test_jump/full_2000.pt