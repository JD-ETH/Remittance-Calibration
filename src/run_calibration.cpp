#include <string>
#include <iostream>
#include <exception>
#include <pcl/console/parse.h>
#include <boost/filesystem.hpp>
#include <glog/logging.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include "remittance_calib/calibrator.hpp"

namespace bf = boost::filesystem;
namespace remittance_calib
{
/**
 * \brief information needed to convert txt to binary
 */
struct Options
{
    std::string input_ply ; //< Folder name
    std::string calib_file; //< File used for mapping
    double epsilon, voxel_size, std_var; // Some setups
    float dist_thresh;
};

Options parseOptions(int argc, char** argv)
{
    Options options;
    pcl::console::parse_argument (argc, argv, "-i", options.input_ply);
    pcl::console::parse_argument (argc, argv, "-c", options.calib_file);

    pcl::console::parse_argument (argc, argv, "-s", options.std_var);
    pcl::console::parse_argument (argc, argv, "-e", options.epsilon);
    pcl::console::parse_argument (argc, argv, "-v", options.voxel_size);


    pcl::console::parse_argument (argc, argv, "-d", options.dist_thresh);

    return options;

}
}

int main(int argc, char** argv)
{
    FLAGS_stderrthreshold = 0;
    FLAGS_colorlogtostderr = true;
    google::InitGoogleLogging(argv[0]);

    auto options =  remittance_calib::parseOptions(argc,argv);

    remittance_calib::Calibrator calib(options.voxel_size, options.std_var, options.epsilon);
    auto cloud = remittance_calib::loadCloud(options.input_ply,options.dist_thresh);
    LOG(INFO) << "Loaded cloud size " << cloud->size();
    calib.loadCloud(cloud);
    auto res = calib.run();
    LOG(INFO) << "Converged";
    remittance_calib::saveMappings(options.calib_file,res);
    


    return 0;
}
