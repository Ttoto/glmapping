#ifndef CYLINDERICAL_CELL_H
#define CYLINDERICAL_CELL_H

#include <utils/include/all_utils.h>


struct CYLINDRICAL_CELL{
  int idx;
  int idx_rho;
  int idx_phi;
  int idx_z;
  Vec3 center_pt;
  Vec3 sampled_xyz;
  double raycasting_z_over_rho;
  bool is_occupied;
};

#endif // CYLINDERICAL_CELL_H
