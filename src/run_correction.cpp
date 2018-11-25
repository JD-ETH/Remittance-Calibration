#include <string>
#include <iostream>
#include <exception>
#include <pcl/console/parse.h>
#include <boost/filesystem.hpp>
#include <glog/logging.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include "remittance_calib/io.hpp"

namespace bf = boost::filesystem;
namespace remittance_calib
{
/**
 * \brief information needed to convert txt to binary
 */
struct Options
{
    std::string input_folder ; //< Folder name
    std::string output_folder ; //< Output folder to be saved
    std::string calib_file; //< File used for mapping
    float dist_thresh, correct_thresh; // thresh
    std::vector<std::string> plyFiles; //< Points needs to be fixed
};

Options parseOptions(int argc, char** argv)
{
    Options options;
    pcl::console::parse_argument (argc, argv, "-i", options.input_folder);
    pcl::console::parse_argument (argc, argv, "-c", options.calib_file);

    pcl::console::parse_argument (argc, argv, "-o", options.output_folder);

    pcl::console::parse_argument (argc, argv, "-d", options.dist_thresh);
    pcl::console::parse_argument (argc, argv, "-r", options.correct_thresh);

    if(options.plyFiles.empty())//If we do not have a ply file list then walk the directory and search for them using the prefix if supplied
    {
      bf::path inputFilesystem(options.input_folder);
      if(!is_directory(inputFilesystem))
      {
          LOG(ERROR) << options.input_folder << " missing " ;
          return options;

      }


      //Loop thru the entries and extract any ending with .ply and starting with the prefix
      for(boost::filesystem::directory_iterator it(inputFilesystem) ; it != bf::directory_iterator() ; ++it )
        if(bf::is_regular_file(it->path()))
        {
          //Bag check
          if(it->path().filename().extension().string() != ".ply")
            continue;

          options.plyFiles.push_back(options.input_folder + it->path().filename().string());
        }

      std::sort(options.plyFiles.begin(), options.plyFiles.end());
    }



    if(options.plyFiles.size() == 0)
    {
        LOG(ERROR) << "No ply found " ;
        return options;
    }
    else
    {
        LOG(INFO) << "Found ply files " << options.plyFiles.size();
    }
    return options;

}
}

int main(int argc, char** argv)
{
    FLAGS_stderrthreshold = 0;
    FLAGS_colorlogtostderr = true;
    google::InitGoogleLogging(argv[0]);

    auto options =  remittance_calib::parseOptions(argc,argv);

    remittance_calib::BeamMappings maps;
    remittance_calib::loadMappings(options.calib_file,maps);
    LOG(INFO) << "Loading complete in total " << maps.size();
    for (const auto file : options.plyFiles)
    {
        auto cloud = remittance_calib::loadFullCloud(file,options.dist_thresh);
        pcl::PointCloud<pcl::PointXYZINormal> res;


        for (auto & pt : cloud->points)
        {
            pcl::PointXYZINormal point;
            point.x = pt.x;
            point.y = pt.y;
            point.z = pt.z;
            point.curvature = static_cast<float>(pt.time);
            int downsized = static_cast<int>(pt.intensity/256);

            if (pt.range > options.correct_thresh || downsized >= 100)
            {
                point.intensity = static_cast<float>(downsized)/256.0f;
            }
            else
            {
                CHECK(pt.ring < maps.size());
                CHECK(maps.at(pt.ring).cols()>downsized);
                point.intensity = static_cast<float>(maps.at(pt.ring)(downsized))/256.0f;
            }

            CHECK(point.intensity < 1.0f) << "Converted intensity must smaller than 1";
            res.push_back(point);
        }

        LOG(INFO) << "Conversion complete for file " << file;

        if (!remittance_calib::saveCloud(options.output_folder +
                        remittance_calib::extractFileName(file) + ".pcd",res))
        {
            LOG(ERROR) << "Save failed for " << file;
        }
    }



    return 0;
}
