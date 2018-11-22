#include "remittance_calib/calibrator.hpp"

namespace remittance_calib
{
    BeamMappings Calibrator::run()
    {
        int count = 0;
        while (e_step()>convergence_)
        {
            double error = m_step();
            LOG(INFO) << "Iteration " << ++count << " Has M step error " << error;
        }
        BeamMappings res ;
        for (const auto & beam : beam_model)
        {
            res.emplace_back(beam.getMapping());
        }
        return res;
    }

    void Calibrator::loadCloud(pcl::PointCloud<PointXYZIR>::Ptr cloud)
    {
        measurements_ = loadMeasurement(cloud, voxel_size);

        LOG(INFO) << "Downsampled size of voxel " << cloud->size();
        cell_model.resize(cloud->size());

        // Init cell
        for (auto & cell_prob : cell_model)
        {
            cell_prob = Eigen::Matrix<double,256,1>::Ones() * uniform_dist;
        }

        // Init beam
        int num_of_rings = 0;
        for (const auto & m: measurements_) num_of_rings = std::max(num_of_rings,m.b);
        num_of_rings ++;
        LOG(INFO) << "Number of beams " << num_of_rings;
        for (int beam_i = 0 ; beam_i < num_of_rings ; beam_i++)
        {
            beam_model.emplace_back(std_var,epsilon);
        }


    }

    double Calibrator::e_step()
    {
        auto buffered = cell_model;
        for (const auto & m : measurements_)
        {
            CHECK(m.k < cell_model.size()) << " Index too high";
            cell_model.at(m.k) += beam_model.at(m.b).at(m.a);
        }

        double diff = 0.0;
        for (int i = 0 ; i < cell_model.size(); i++)
        {
            cell_model.at(i) /= cell_model.at(i).sum(); // Average
            diff += (cell_model.at(i)-buffered.at(i)).norm();
        }

        return diff/cell_model.size();

    }

    double Calibrator::m_step()
    {
        auto buffered = beam_model;
        BeamCountings countings(beam_model.size());
        for (auto counting: countings) counting.setZero();

        for (const auto & m : measurements_)
        {
            countings.at(m.b).col(m.a) += cell_model.at(m.k);
        }
        double diff = 0;
        for (uint i = 0 ; i < countings.size() ; i++)
        {
            beam_model.at(i) = BeamProbability(countings.at(i));
            diff += (beam_model.at(i).probability-buffered.at(i).probability).norm();
        }
        return diff/countings.size();
    }
}
