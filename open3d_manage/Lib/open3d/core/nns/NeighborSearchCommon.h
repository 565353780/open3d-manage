#pragma once

enum Metric { L1, L2, Linf };

struct NanoFlannIndexHolderBase {
  virtual ~NanoFlannIndexHolderBase() {}
};
