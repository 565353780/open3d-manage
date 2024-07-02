# Open3D Manage

## Download

```bash
https://github.com/isl-org/Open3D/releases/download/v0.18.0/open3d-devel-linux-x86_64-pre-cxx11-abi-0.18.0.tar.xz
->./3rd/open3d-devel-linux/
https://github.com/isl-org/Open3D/releases/download/v0.18.0/open3d-devel-windows-amd64-0.18.0.zip
->./3rd/open3d-devel-win/
```

backup url

```bash
https://rec.ustc.edu.cn/share/4ba92ac0-362e-11ef-8657-d7763981ad73
```

## Setup

```bash
conda create -n o3d python=3.10
conda activate o3d
./setup.sh
```

## Run

```bash
python demo.py
```

## Details

### BSpline Surface

```bash
B样条曲面为物体表面区域的参数化表达，它将物体表面区域（例如一个矩形区域）沿两个方向（例如X轴和Y轴方向）进行参数化，并将这两个参数化方向称为u,v方向，其中u,v取值范围一般为[0,1]
```

### Inputs

```bash
sigma_d: 点云降噪时邻域点的距离的权重系数，越大则越优先考虑近邻点
sigma_n: 点云降噪时邻域点的法向的权重系数，越大则越优先考虑近邻点

curvature_knn_num: 曲率估计时考虑的邻域点数量，越大则估计的曲率越光滑，但耗时越长
filter_knn_num: 点云降噪时考虑的邻域点数量，越大则降噪后的点云整体越光滑，但耗时越长

need_smooth: 法向估计时是否进行进一步光顺，一般默认进行光顺，以得到更平滑的结果

points: 待降噪点云，数据排列需为 [x1, y1, z1, x2, y2, z2, ...]，点集中的各点无先后顺序要求
```

### Outputs

```bash
const std::vector<float> denoised_points
降噪后的点云，点的顺序保持与输入点云一致
```

## Enjoy it~
