#ifndef REMITTANCE_CALIB_CALIB_HPP_
#define REMITTANCE_CALIB_CALIB_HPP_
#include "remittance_calib/types.hpp"

#include "remittance_calib/io.hpp"

namespace remittance_calib
{
    class Calibrator
    {
    public:
        Calibrator() = delete;
        Calibrator(double voxel_size, double std_var, double epsilon):
                voxel_size(voxel_size), std_var(std_var), epsilon(epsilon){}
        BeamMappings run();
        void loadCloud(pcl::PointCloud<PointXYZIR>::Ptr cloud);
    private:
        static constexpr double convergence_ = 1e-7 ;
        static constexpr double uniform_dist = 1.0/256.0 ;
        BeamModel beam_model;
        CellModel cell_model;
        double e_step();
        double m_step();
        double voxel_size ;
        double std_var ;
        double epsilon ;
        Measurements measurements_;
    };
}


#endif
