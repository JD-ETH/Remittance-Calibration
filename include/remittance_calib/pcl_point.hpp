#define PCL_NO_PRECOMPILE
#ifndef REMITTANCE_CALIB_PCL_POINT_HPP_
#define REMITTANCE_CALIB_PCL_POINT_HPP_
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/filters/impl/voxel_grid.hpp>
#include <pcl/impl/pcl_base.hpp>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/search/impl/search.hpp>
#include <pcl/PointIndices.h>
#include <pcl/pcl_base.h>
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

  struct PointFull
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
          unsigned char planarity;  // This is 8 bit, number of ring
    			float incidence_angle; // This is 32 bit
    			float range; // This is 32 bit again
    		};
    	};


    	union
    	{
    		struct
    		{
    			double time;
    		};
    		double data_t[2]; // ensuring SSE alignment
    	};

    	EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
    }EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

  // registration of the point (apparently: actual declaration of the point)
    POINT_CLOUD_REGISTER_POINT_STRUCT(PointFull,
    		(float, x, x)
    		(float, y, y)
    		(float, z, z)
  	        (unsigned char, planarity, planarity)
  	        (unsigned char, ring, ring_number)
    		(unsigned short, intensity, intensity)
    		(float, incidence_angle, incidence_angle)
    		(float, range, range)
    		(double, time, time)
    )


#endif
