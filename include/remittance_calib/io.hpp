#ifndef REMITTANCE_CALIB_IO_HPP_
#define REMITTANCE_CALIB_IO_HPP_

#include <iostream>
#include <fstream>
#include <string>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include "remittance_calib/types.hpp"

#include "remittance_calib/pcl_point.hpp"
namespace remittance_calib
{
    pcl::PointCloud<PointXYZIR>::Ptr loadCloud(const std::string & filename, float dist_thresh = 40.0);
    // Load from point complete
    pcl::PointCloud<PointFull>::Ptr loadFullCloud(const std::string & filename, float dist_thresh = 40.0);
    bool saveCloudAsFull(const std::string & filename,const pcl::PointCloud<PointFull> & cloud);
    pcl::PointCloud<PointFull>::Ptr loadPly_PointFull_manual(std::string file,float dist_thresh = 40.0);

    Measurements loadMeasurement(pcl::PointCloud<PointXYZIR>::Ptr & cloud, double voxel_size=0.25);
    // Save to point XYZINormal for experiments
    bool saveCloud(const std::string & filename,const pcl::PointCloud<pcl::PointXYZINormal> & cloud);

    // Get ind

    inline int find_voxel( pcl::VoxelGrid<PointXYZIR> & voxelgrid,
      float x, float y, float z){
        return voxelgrid.getCentroidIndexAt(voxelgrid.getGridCoordinates(x,y,z));
     }

     inline std::string extractFileName(const std::string & filename){
       std::size_t begin = filename.find_last_of("/\\");
       std::size_t end = filename.find_last_of(".");
       if (begin == std::string::npos || end == std::string::npos || end-begin <= 0){
         std::cerr << "File name is irregular" << filename << '\n';
         return filename;
       }
       return filename.substr(begin+1,end-begin-1);
     }
     void align(pcl::PointCloud<PointFull>::Ptr ref,pcl::PointCloud<PointFull>::Ptr src, Eigen::Matrix4f hint );

     void saveMappings(const std::string & filename, const BeamMappings & mappings);
     void loadMappings(const std::string & filename, BeamMappings & mappings);
     void saveProbability(const std::string & filename, const BeamModel & beam_probs);
     // void saveBeamcounting(const std::string & filename, const BeamCountings & counts)

}
#endif
