now=$(date +"%y%m%d%H%M%S")

python3 - << END
import torch

# GPU가 사용 가능한지 확인
if torch.cuda.is_available():
    num_gpu = torch.cuda.device_count()

    for i in range(num_gpu):
        gpu = torch.cuda.get_device_properties(i)
        print(f'---------------------------------------------------------')
        print(f'[GPU list]')
        print(f" - GPU {i}: {gpu.name}, Memory: {gpu.total_memory / 1024**3}GB")
        print(f'---------------------------------------------------------')        
else:
    print("CUDA is not available.")
END

read -p $'\e[33m>> Choose GPU number: \e[0m' GPUNUM
set -e

echo $'\e[33m[NOTICE] The training data will be stored in \e[0m[raisimGymTorch/data/rsg_a1_task/'"$now"$'] \e[33mdirectory! \e[0m'

cd ./raisimGymTorch/raisimGymTorch/env/envs/rsg_a1_task
set -e
python runner.py --name random --gpu $GPUNUM --exptid $now
