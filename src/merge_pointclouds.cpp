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
    std::string target_ply, source_ply ; //< Folder name
    std::string output_folder ; //< Output folder to be saved
    float x,y,z,qw,qx,qy,qz;
};

Options parseOptions(int argc, char** argv)
{
    Options options;
    pcl::console::parse_argument (argc, argv, "-t", options.target_ply);
    pcl::console::parse_argument (argc, argv, "-s", options.source_ply);
    pcl::console::parse_argument (argc, argv, "-o", options.output_folder);
    pcl::console::parse_argument (argc, argv, "-x", options.x);
    pcl::console::parse_argument (argc, argv, "-y", options.y);
    pcl::console::parse_argument (argc, argv, "-z", options.z);
    pcl::console::parse_argument (argc, argv, "-qw", options.qw);
    pcl::console::parse_argument (argc, argv, "-qx", options.qx);
    pcl::console::parse_argument (argc, argv, "-qy", options.qy);
    pcl::console::parse_argument (argc, argv, "-qz", options.qz);


    return options;

}
}

int main(int argc, char** argv)
{
    FLAGS_stderrthreshold = 0;
    FLAGS_colorlogtostderr = true;
    google::InitGoogleLogging(argv[0]);

    auto options =  remittance_calib::parseOptions(argc,argv);


    auto cloud_ref = remittance_calib::loadFullCloud(options.target_ply,100.0);
    auto cloud_src = remittance_calib::loadFullCloud(options.source_ply,100.0);
    pcl::PointCloud<PointFull> res;

    Eigen::Matrix4f T  = Eigen::Matrix4f::Identity();
    T(0,3) = options.x; T(1,3) = options.y; T(2,3) = options.z;
    Eigen::Quaternionf q(options.qw, options.qx, options.qy, options.qz);
    T.block<3,3>(0,0) = q.toRotationMatrix();
    remittance_calib::align(cloud_ref, cloud_src, T);

    res = *cloud_ref + *cloud_src;
    LOG(INFO) << "Combined number of points " << res.size() << "Moved " << cloud_src->size();

    if (!remittance_calib::saveCloudAsFull(options.output_folder +"merged.ply",res))
    {
        LOG(ERROR) << "Save failed for " << options.output_folder +"merged.ply";
    }




    return 0;
}
