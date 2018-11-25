#include "remittance_calib/calibrator.hpp"

namespace remittance_calib
{
    BeamMappings Calibrator::run()
    {
        int count = 0;
        while (e_step()>convergence_ && count < 15)
        {
            double error = m_step();
            saveProbability("/home/guo104/data/prob"+std::to_string(count)+".txt",beam_model);
            LOG(WARNING) << "Iteration " << ++count << " Has M step error " << error;
            BeamMappings res ;
            for (const auto & beam : beam_model)
            {
                res.emplace_back(beam.getMapping());
            }
            saveMappings("/home/guo104/data/map"+std::to_string(count)+".txt",res);
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
        LOG(INFO) << "Loaded measurements. Downsampled size of voxel " << cloud->size();
        cell_model.resize(cloud->size());

        // Init cell
        for (auto & cell_prob : cell_model)
        {
            cell_prob = Eigen::Matrix<double,MAX_REMITTANCE_READING,1>::Ones() * uniform_dist;
        }
        LOG(INFO) << "Initialized Cell Probability as uniform " << cell_model.size();

        // Init beam
        int num_of_rings = 0;
        for (const auto & m: measurements_) num_of_rings = std::max(num_of_rings,m.b);
        num_of_rings ++;
        LOG(INFO) << "Number of beams " << num_of_rings;

        for (int beam_i = 0 ; beam_i < num_of_rings ; beam_i++)
        {
            beam_model.emplace_back(MAX_REMITTANCE_READING,std_var,epsilon);
        }

        LOG(INFO) << "Initialization completed";
    }

    double Calibrator::e_step()
    {
        LOG(INFO) << "RUNNING E STEP " ;
        auto buffered = cell_model;
        for (auto & cell : cell_model) cell.setZero();
        int counter = 0 ;
        for (const auto & m : measurements_)
        {

            CHECK(m.k < cell_model.size()) << " Index too high";
            cell_model.at(m.k) += beam_model.at(m.b).atLog(m.a);

        }
        for (auto & cell : cell_model)
        {
            auto val = cell.maxCoeff();
            for (uint i = 0 ; i < MAX_REMITTANCE_READING ; i++)
            {
                if (cell(i)-val >=std::log(precision_)-std::log(MAX_REMITTANCE_READING))
                {
                    cell(i) = std::exp(cell(i)-val);
                }
                else
                {
                    cell(i) = 0;
                }
            }
        }
        double diff = 0.0;
        for (int i = 0 ; i < cell_model.size(); i++)
        {
            cell_model.at(i) /= cell_model.at(i).sum(); // Average
            double err = (cell_model.at(i)-buffered.at(i)).norm();
            if (err != err)
            {
                LOG(ERROR) << cell_model.at(i) ;

                throw std::runtime_error(" Illegal value");
            }
            diff += err;
        }

        double res =  diff/cell_model.size();
        LOG(WARNING) << "Current E STEP averaged error" << res;
        return res;
    }

    double Calibrator::m_step()
    {

        LOG(INFO) << "RUNNING M STEP " ;
        auto buffered = beam_model;
        BeamCountings countings(beam_model.size());
        for (auto & counting: countings)
        {
            counting = BeamCounting(MAX_REMITTANCE_READING,MAX_REMITTANCE_READING);
            counting.setZero();
        }
        for (const auto & m : measurements_)
        {

            countings.at(m.b).col(m.a) += cell_model.at(m.k);
        }
        double diff = 0;
        for (uint i = 0 ; i < countings.size() ; i++)
        {
            CHECK_EQ(countings.at(i).rows(), MAX_REMITTANCE_READING);
            beam_model.at(i) = BeamProbability(countings.at(i));
            diff += (beam_model.at(i).probability-buffered.at(i).probability).norm();
        }
        double res =  diff/countings.size();

        return res;
    }
}
