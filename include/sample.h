#pragma once

#include <string>

const bool sampleLocalSurfaceNearVertex(const std::string &mesh_file_path,
    const int &vertex_idx, const int &sample_face_num, const std::string &save_mesh_file_path, const bool &overwrite = false);
