#include <string>
#include <iostream>
#include <exception>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <glog/logging.h>

#include "pcl/point_types.h"
#include "remittance_calib/calibrator.hpp"

namespace bo = boost::program_options;
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
    bo::options_description desc("Allowed Options");
    desc.add_options()
        ("help,h", "show help message")
        ("in,i", boost::program_options::value<std::string>(&options.input_ply), "input ply file ")
        ("file,f", boost::program_options::value<std::string>(&options.calib_file), "calibration file to output")
        ("voxel_size,v", boost::progCalibratorram_options::value<double>(&options.voxel_size)->default_value(0.4), "Voxel size ")
        ("epsilon,e", boost::program_options::value<double>(&options.epsilon)->default_value(0.2), "Random Distrib ")
        ("std_var,s", boost::program_options::value<double>(&options.std_var)->default_value(5), "Standard variation ")
        ("disance,d", boost::program_options::value<float>(&options.dist_thresh)->default_value(40.0), "dist thresh ");;
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
    calib.loadCloud(cloud);
    auto res = calib.run();

    saveMappings(options.calib_file,res);




    return 0;
}
