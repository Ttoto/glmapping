#include "cc_agrid.h"

cc_agrid::cc_agrid()
{

}

cc_agrid::cc_agrid(int Rho, int Phi, int Z, Vec3 sample_xyz)
{
    this->idx_rho = Rho;
    this->idx_phi = Phi;
    this->idx_z = Z;
    this->sample_pt=sample_xyz;

}
