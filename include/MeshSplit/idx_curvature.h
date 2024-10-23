#pragma once

class IdxCurvature {
public:
  IdxCurvature() {};

  IdxCurvature(const int &idx_value, const double &curvature_value);

public:
  int idx;
  double curvature;
};
