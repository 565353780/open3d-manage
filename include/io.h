#pragma once

#include <string>
#include <vector>

const bool savePointsAsPcd(const std::vector<float> &points, const std::string &save_file_path, const bool &overwrite = false);

const std::vector<float> loadPointsFromFile(const std::string &file_path);
