build_folder="build"

if [ -d "$build_folder" ]; then
    echo "Deleting $build_folder..."
    rm -rf "$build_folder"
    echo "$build_folder deleted successfully."
fi

mkdir "$build_folder"
cd "$build_folder"

python_path=$(which python)

set -e

raisimlib_dir=$(pwd)
raisimlib_install_dir="$raisimlib_dir/install"

cmake .. -DCMAKE_INSTALL_PREFIX=$raisimlib_install_dir -DRAISIM_EXAMPLE=ON -DRAISIM_PY=ON -DPYTHON_EXECUTABLE=$python_path

set -e

make install -j4

set -e

echo -e "\e[33m>> Ready for raisimLib!\e[0m"

cd ../raisimGymTorch

if [ -d "$build_folder" ]; then
    echo "Deleting $build_folder..."
    rm -rf "$build_folder"
    echo "$build_folder deleted successfully."
fi

python setup.py develop

set -e

echo -e "\e[33m>> Ready for raisimGymTorch!\e[0m"
