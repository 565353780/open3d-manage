#pragma once

#include "NumCpp.hpp"
#include <open3d/Open3D.h>

open3d::geometry::PointCloud toPcd(const nc::NdArray<float> &points);
