#ifndef REMITTANCE_CALIB_TYPES_HPP_
#define REMITTANCE_CALIB_TYPES_HPP_
#define PCL_NO_PRECOMPILE
#include <glog/logging.h>
#include <Eigen/Dense>
#include "pcl/point_types.h"
#include "remittance_calib/pdf.hpp"
// PCL point type used here
struct PointXYZIR
 {
    // Standard Point
    PCL_ADD_POINT4D;

  	union
  	{
      float data_c[4]; // ensuring SSE alignment
  		struct
  		{
  			unsigned char ring;  // This is 8 bit, number of ring
  			unsigned short intensity; // This is 16 bit
            float range ;
        };
  	};


  	EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
  }EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

// registration of the point (apparently: actual declaration of the point)
  POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIR,
  		(float, x, x)
  		(float, y, y)
  		(float, z, z)
	    (unsigned char, ring, ring)
  		(unsigned short, intensity, intensity)
        (float, range, range)
  )



namespace remittance_calib
{
using CellProbability = Eigen::Matrix<double,256,1>;
using BeamCounting = Eigen::Matrix<double,256,256>;
using BeamMapping = Eigen::Matrix<int,256,1>; //< Map to 0 till 255


// Rows Measured 0 till 255
// Columns Real Value, 0 till 255
struct BeamProbability
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    BeamProbability(float var = 1.0, float epsilon = 0.1);

    BeamProbability(BeamCounting in):probability(in){normalize();}

    Eigen::Matrix<double,256,256> probability;

    BeamMapping getMapping() const;

    void normalize()
    {
        Eigen::Matrix<double,256,1> row_wise_sum = probability.rowwise().sum();
        probability.array().colwise() /= row_wise_sum.array();
        CHECK_EQ(probability.row(0).sum(),1);
    }

    CellProbability at(int measured){ return probability.col(measured);}
};

using BeamModel = std::vector<BeamProbability>;
using BeamMappings = std::vector<BeamMapping,Eigen::aligned_allocator<BeamMapping> > ;
struct Measurement
{
    Measurement(int a, int b, int k):a(a),b(b),k(k){}
    // measured, beam, index of
    int a, b, k;
};
using Measurements = std::vector<Measurement>;
using CellModel = std::vector<CellProbability,Eigen::aligned_allocator<CellProbability> > ;
using BeamCountings = std::vector<BeamCounting, Eigen::aligned_allocator<BeamCounting> > ;
}
#endif
