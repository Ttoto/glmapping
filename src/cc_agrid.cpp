#include "cc_agrid.h"

cc_agrid::cc_agrid()
{

}

cc_agrid::cc_agrid(int Rho, int Phi, int Z, Vec3 vis_xyz)
{
    this->idx_rho = Rho;
    this->idx_phi = Phi;
    this->idx_z = Z;
    this->vis_pt=vis_xyz;

}
