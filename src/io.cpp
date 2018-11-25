#include "remittance_calib/io.hpp"

namespace remittance_calib
{

    pcl::PointCloud<PointFull>::Ptr loadFullCloud(const std::string & filename, float dist_thresh )
    {

        if (extractFileName(filename).substr(0,6) == "result")
        {
            return loadPly_PointFull_manual(filename,dist_thresh);
        }

        pcl::PointCloud<PointFull>::Ptr cloud(new pcl::PointCloud<PointFull>);
        pcl::io::loadPLYFile(filename,*cloud);
        pcl::PointCloud<PointFull>::Ptr res(new pcl::PointCloud<PointFull>);
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
    void align(pcl::PointCloud<PointFull>::Ptr ref,pcl::PointCloud<PointFull>::Ptr src, Eigen::Matrix4f hint )
    {
        pcl::PointCloud<PointFull>::Ptr ref_down(new pcl::PointCloud<PointFull>);
        pcl::PointCloud<PointFull>::Ptr src_down(new pcl::PointCloud<PointFull>);

        pcl::PointCloud<PointFull>::Ptr tmp(new pcl::PointCloud<PointFull>);
        pcl::VoxelGrid<PointFull> grid;
        grid.setLeafSize(0.25,0.25,0.25);
        grid.setInputCloud(src);
        grid.filter(*src_down);
        grid.setInputCloud(ref);
        grid.filter(*ref_down);

        // Run the alignement
        pcl::IterativeClosestPoint<PointFull, PointFull> reg;
        reg.setMaxCorrespondenceDistance(2.0);
        reg.setMaximumIterations (30);
        // Set the maximum distance between two correspondences (src<->tgt) to 10cm
        // Note: adjust this based on the size of your datasets
        reg.setInputSource(src_down);
        reg.setInputTarget(ref_down);
        reg.align(*tmp,hint);

        pcl::transformPointCloud(*src,*src,hint);
        for (auto & pt : src->points) pt.ring += 16;


        LOG(INFO) << "Alignment has converged:" << reg.hasConverged() << " score: " <<
          reg.getFitnessScore() << std::endl;

    }
    pcl::PointCloud<PointXYZIR>::Ptr loadCloud(const std::string & filename, float dist_thresh)
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
    Measurements loadMeasurement(pcl::PointCloud<PointXYZIR>::Ptr & cloud,double voxel_size)
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
        //std::vector<size_t> counter(257,0);
        for (const auto pt : cloud->points)
        {
            int a = static_cast<int>(pt.intensity/256);
            int b = static_cast<int>(pt.ring);
            int k = find_voxel(grid,pt.x,pt.y,pt.z);
            if (a>=100)
            {
                continue;
            }
            CHECK(k>=0) << "Did not find corresponding voxel";
            CHECK(a<256 && a >=0) << "Measured intensity is wrong value";
            CHECK(b>=0) << "Ring index is wrong";
            results.emplace_back(a,b,k);
            // counter.at(a)++;
        }
        LOG(INFO) << "Gathered measurements " << results.size();
        cloud.swap(tmpcloud);
        //for (int i = 0 ; i < 257 ;  i++) LOG(INFO) << "Measuring " << i << " : " <<counter.at(i) << " times";
        return results;

    }

    // mapping io
    void saveMappings(const std::string & filename, const BeamMappings & mappings)
    {
        std::ofstream file(filename.c_str());
        if (file.is_open())
        {
          file << mappings.size() << " " << mappings.at(0).cols() <<"\n";
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
        getline(ifs, line);
        std::istringstream iss(line);
        int num_of_rings, size_of_mapping;
        iss>>num_of_rings;
        iss>> size_of_mapping;
        LOG(INFO) << " There are " << num_of_rings << "rings and correspondences" << size_of_mapping;
      for (int k = 0 ; k < num_of_rings ; k++)
      {
          getline(ifs, line);
          std::istringstream iss(line);

          BeamMapping mapping(size_of_mapping);
          for (int i = 0 ; i < size_of_mapping ; i++)
          {
              iss >> mapping(i);

          }
          mappings.push_back(mapping);
      }

    }

    void saveProbability(const std::string & filename,  const BeamModel & beam_probs)
    {
        std::ofstream file(filename.c_str());
        if (file.is_open())
        {

          for (const auto beam:  beam_probs)
          {
              file << beam.probability << '\n';
          }
        }
    }
    bool saveCloud(const std::string & filename,const pcl::PointCloud<pcl::PointXYZINormal> & cloud)
    {
        LOG(INFO) << "Saving under " << filename;
        return pcl::io::savePCDFileBinary(filename,cloud)!= -1;
    }
    bool saveCloudAsFull(const std::string & filename,const pcl::PointCloud<PointFull> & cloud)
    {
        LOG(INFO) << "Saving under " << filename;
        return pcl::io::savePLYFileBinary(filename,cloud)!= -1;
    }
    pcl::PointCloud<PointFull>::Ptr loadPly_PointFull_manual(std::string file,float dist_thresh){
    /*

      Thus we follow this sequence and hope nothing ever changes or breaks
    */
      size_t numOfVertices;
      std::string::size_type sz;   // alias of size_t
      std::ifstream inputPly;
      inputPly.open(file.c_str(),std::ios::binary);
      if (!inputPly.is_open())
      {
        std::cerr << "Couldn't open " << file << '\n';
        exit(1);
      }

      std::string line;
      while (line.find("element vertex") != 0){
        getline(inputPly,line);
      }
      numOfVertices = std::stol(line.substr(15),&sz);

      getline(inputPly,line);

      while (line.compare("end_header") != 0){
    		getline(inputPly, line);
      }

      LOG(WARNING) << "Manually parsing";

      // Skip through the end of header
      pcl::PointCloud<PointFull>::Ptr cloud (new pcl::PointCloud<PointFull>());
      PointFull single_point;
      // BAD PRACTICE reinterpret_cast means force interpret
      while (numOfVertices--){

        inputPly.read(reinterpret_cast<char*>( &single_point.x), sizeof(float));//std::cout << float(x)<< " ";
        inputPly.read(reinterpret_cast<char*>( &single_point.y), sizeof(float));//std::cout << float(y)<< " ";
        inputPly.read(reinterpret_cast<char*>( &single_point.z), sizeof(float));//std::cout << float(z)<< " ";
        inputPly.read((char *) &single_point.intensity, sizeof(single_point.intensity));// std::cout << int(single_point.intensity) << " ";
        inputPly.read((char *) &single_point.incidence_angle, sizeof(single_point.incidence_angle));//std::cout << (single_point.incidence_angle)<< " ";

        inputPly.read((char *) &single_point.range, sizeof(single_point.range));//std::cout << (single_point.range)<< " ";
        inputPly.read((char *) &single_point.planarity, sizeof(single_point.planarity));//std::cout << (int)single_point.planarity << " ";
        inputPly.read((char *) &single_point.ring, sizeof(single_point.ring));//std::cout << int(single_point.ring)<< " ";
        inputPly.read((char *) &single_point.time, sizeof(single_point.time));//std::cout << single_point.time << " \n";

        if (single_point.range <= dist_thresh)
            cloud->push_back(single_point);
      }
      return cloud;

    }

}
