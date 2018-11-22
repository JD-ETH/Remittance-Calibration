#include <string>
#include <iostream>
#include <exception>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <glog/logging.h>

#include "pcl/point_types.h"
#include "remittance_calib/io.hpp"

namespace bo = boost::program_options;
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
    float dist_thresh; // thresh
    std::vector<std::string> plyFiles; //< Points needs to be fixed
};

Options parseOptions(int argc, char** argv)
{
    Options options;
    bo::options_description desc("Allowed Options");
    desc.add_options()
        ("help,h", "show help message")
        ("in,i", boost::program_options::value<std::string>(&options.input_folder), "input folder")
        ("out,o", boost::program_options::value<std::string>(&options.output_folder), "output folder loc")
        ("file,f", boost::program_options::value<std::string>(&options.calib_file), "calibration file used for mapping")
        ("disance,d", boost::program_options::value<float>(&options.dist_thresh)->default_value(40.0), "dist thresh ");
    bo::variables_map vm;
    try
    {
      bo::store(bo::command_line_parser(argc, argv).options(desc).run(), vm);
      bo::notify(vm);
    }
    catch(bo::invalid_command_line_syntax& e)
    {
      throw std::runtime_error(e.what());
    }
    catch(bo::unknown_option& e)
    {
      throw std::runtime_error(e.what());
    }



    //Print help and exit
    if(vm.count("help") || vm.size() == 0)
    {
      std::cout << desc << std::endl;
      exit(0);
    }

    if(options.plyFiles.empty())//If we do not have a ply file list then walk the directory and search for them using the prefix if supplied
    {
      bf::path inputFilesystem(options.input_folder);
      if(!is_directory(inputFilesystem))
        throw std::exception("'" + inputDirectory + "' is not a directory");

      //Loop thru the entries and extract any ending with .ply and starting with the prefix
      for(boost::filesystem::directory_iterator it(inputFilesystem) ; it != bf::directory_iterator() ; ++it )
        if(bf::is_regular_file(it->path()))
        {
          //Bag check
          if(it->path().filename().extension().string() != ".ply")
            continue;

          //Prefix check
          if(inputPrefix != "" && it->path().filename().string().find(inputPrefix) != 0)
            continue;

          options.plyFiles.push_back(inputDirectory + it->path().filename().string());
        }

      std::sort(options.plyFiles.begin(), options.plyFiles.end());
    }



    if(options.plyFiles.size() == 0)
      throw std::exception("No plys found");
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

    for (const auto file : options.plyFiles)
    {
        auto cloud = remittance_calib::loadCloud(file,options.dist_thresh);
        pcl::PointCloud<pcl::PointXYZINormal> res;
        for (auto & pt : cloud->points)
        {
            pcl::PointXYZINormal point;
            point.x = pt.x;
            point.y = pt.y;
            point.z = pt.z;
            int downsized = static_cast<int>(pt.intensity/256);
            point.intensity = static_cast<float>(maps.at(pt.ring).at(downsized))/256.0f;
            CHECK(point.intensity < 1.0f) << "Converted intensity must smaller than 1";
            res.push_back(point);
        }



        if (!remittance_calib::saveCloud(options.output_folder + extractFileName(file),res))
        {
            LOG(ERROR) << "Save failed for " << file;
        }
    }



    return 0;
}
