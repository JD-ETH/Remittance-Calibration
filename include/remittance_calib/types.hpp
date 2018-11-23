#ifndef REMITTANCE_CALIB_TYPES_HPP_
#define REMITTANCE_CALIB_TYPES_HPP_

#include <glog/logging.h>
#include <Eigen/Dense>
#include <pcl/cloud_iterator.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include "remittance_calib/pcl_point.hpp"
#include "remittance_calib/pdf.hpp"

namespace remittance_calib
{
using CellProbability = Eigen::VectorXd;
using BeamCounting = Eigen::MatrixXd;
using BeamMapping = Eigen::VectorXi; //< Map to 0 till 255


// Rows Measured 0 till 255
// Columns Real Value, 0 till 255
struct BeamProbability
{


    BeamProbability(double var = 1.0, double epsilon = 0.1);

    BeamProbability(BeamCounting in):probability(in){normalize();}

    Eigen::MatrixXd probability;

    BeamMapping getMapping() const;

    void normalize()
    {
        CHECK_EQ(probability.rows(),256);
        Eigen::VectorXd row_wise_sum = probability.rowwise().sum();
        probability.array().colwise() /= row_wise_sum.array();
        CHECK_NEAR(probability.row(0).sum(),1.0, 1e-4);
    }

    CellProbability at(int measured){ return probability.col(measured);}
};

using BeamModel = std::vector<BeamProbability>;
using BeamMappings = std::vector<BeamMapping> ;
struct Measurement
{
    Measurement(int a, int b, int k):a(a),b(b),k(k){}
    // measured, beam, index of
    int a, b, k;
};
using Measurements = std::vector<Measurement>;
using CellModel = std::vector<CellProbability > ;
using BeamCountings = std::vector<BeamCounting> ;
}
#endif
