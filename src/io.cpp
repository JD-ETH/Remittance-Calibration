#include "remittance_calib/io.hpp"

namespace remittance_calib
{


    pcl::PointCloud<PointXYZIR>::Ptr loadCloud(const std::string & filename, double dist_thresh)
    {
        pcl::PointCloud<PointXYZIR>::Ptr cloud(new pcl::PointCloud<PointXYZIR>);
        pcl::io::loadPLYFile(filename,*cloud);
        pcl::PointCloud<PointXYZIR>::Ptr res(new pcl::PointCloud<PointXYZIR>);
        for (const auto & pt : cloud->points)
        {
            if (pt.range <= dist_thresh )
            {
                res->push_back(pt);
            }
        }
        LOG(INFO) << " From original points " << cloud->size() << " now filtered " << res->size();
        return res;
    }
    // Cut the bull shit and assume its PointComplete
    Measurements loadMeasurement(pcl::PointCloud<PointXYZIR>::Ptr & cloud, double voxel_size)
    {
        CHECK_NOTNULL(cloud);
        // Put in bins
        pcl::VoxelGrid<PointXYZIR> grid;
        grid.setInputCloud(cloud);
        grid.setLeafSize(voxel_size,voxel_size,voxel_size);
        grid.setSaveLeafLayout(true);
        grid.setDownsampleAllData(false);
        pcl::PointCloud<PointXYZIR>::Ptr tmpcloud(new pcl::PointCloud<PointXYZIR>);
        grid.filter(*tmpcloud);

        // Now put in measurement
        Measurements results;
        for (const auto pt : cloud->points)
        {
            int a = static_cast<int>(pt.intensity/256);
            int b = static_cast<int>(pt.ring);
            int k = find_voxel(grid,pt.x,pt.y,pt.z);
            CHECK(k>=0) << "Did not find corresponding voxel";
            CHECK(a<256 && a >=0) << "Measured intensity is wrong value";
            CHECK(b>=0) << "Ring index is wrong";
            results.emplace_back(a,b,k);
        }
        LOG(INFO) << "Gathered measurements " << results.size();
        cloud.swap(tmpcloud);
        return results;

    }

    // mapping io
    void saveMappings(const std::string & filename, const BeamMappings & mappings)
    {
        std::ofstream file(filename.c_str());
        if (file.is_open())
        {
          file << mappings.size() << "\n";
          for (const auto mapping:  mappings)
          {
              file << mapping << '\n';
          }
        }
    }
    void loadMappings(const std::string & filename,  BeamMappings & mappings)
    {
        std::ifstream ifs(filename.c_str(), std::ios::in);
        if(!ifs)
        {
          LOG(ERROR)<< "Failed to open mapping file: " << filename;
          return ;
        }
        mappings = BeamMappings();
        CHECK(ifs.is_open());
        std::string line;
        while (!ifs.eof())
        {
          getline(ifs, line);
          if (line.empty()) continue;
          std::istringstream iss(line);
          int num_of_rings;
          iss>>num_of_rings;
          for (int k = 0 ; k < num_of_rings ; k++)
          {
              BeamMapping mapping;
              for (int i = 0 ; i < 256u ; i++)
              {
                  iss >> mapping(i);
              }
              mappings.push_back(mapping);
          }
        }
    }
    bool saveCloud(const std::string & filename,const pcl::PointCloud<pcl::PointXYZINormal> & cloud)
    {
        LOG(INFO) << "Saving under " << filename;
        return pcl::io::savePCDFileBinary(filename,cloud)!= -1;
    }
}
