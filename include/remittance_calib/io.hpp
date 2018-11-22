#ifndef REMITTANCE_CALIB_IO_HPP_
#define REMITTANCE_CALIB_IO_HPP_

#include <iostream>
#include <fstream>
#include <string>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include "remittance_calib/types.hpp"

namespace remittance_calib
{
    pcl::PointCloud<PointXYZIR>::Ptr loadCloud(const std::string & filename, double dist_thresh = 40.0);
    // Load from point complete
    Measurements loadMeasurement(pcl::PointCloud<PointXYZIR>::Ptr cloud, double voxel_size=0.25);
    // Save to point XYZINormal for experiments
    bool saveCloud(const std::string & filename,const pcl::PointCloud<pcl::PointXYZINormal> & cloud)
    {
        LOG(INFO) << "Saving under " << filename;
        return pcl::io::savePCDFileBinary(filename,cloud)!= -1;
    }
    // Get ind
    template <typename PointT>
    inline int find_voxel(const pcl::VoxelGrid<PointT> & voxelgrid,
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

     void saveMappings(const std::string & filename, const BeamMappings & mappings);
     void loadMappings(const std::string & filename, BeamMappings & mappings);

}
#endif
