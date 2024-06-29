#pragma once

#include "NumCpp.hpp"
#include <memory>
#include <open3d/Open3D.h>

std::shared_ptr<open3d::geometry::PointCloud>
toPcd(const nc::NdArray<float> &points);
