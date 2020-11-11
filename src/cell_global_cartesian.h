#ifndef CARTESIAN_CELL_H
#define CARTESIAN_CELL_H
#include <utils/include/all_utils.h>


struct CARTESIAN_CELL{
  int idx_x;
  int idx_y;
  int idx_z;
  Vec3  center_pt;
  bool  is_occupied;
  float log_odds;
  float sd;
};

#endif // CYLINDERICAL_CELL_H
