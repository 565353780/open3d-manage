rm -rf build

mkdir build
cd build
cmake \
	-DCMAKE_PREFIX_PATH=$(python3 -c 'import torch;print(torch.utils.cmake_prefix_path)') \
	-DOpen3D_ROOT=../3rd/open3d-devel-linux-x86_64-cxx11-abi-0.18.0/ \
	-DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
	..
make -j
