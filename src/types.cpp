#include "remittance_calib/types.hpp"


namespace remittance_calib
{
    BeamProbability::BeamProbability(BeamCounting in)
    {
        CHECK_EQ(in.rows(),256);
        probability = in;
        // Eigen::RowVectorXd col_wise_sum = probability.colwise().sum();
        // for (int i = 0 ; i < 256 ; i++)
        // {
        //     if (col_wise_sum(i) == 0) col_wise_sum(i) =1;
        // }
        // probability.array().rowwise() /= col_wise_sum.array();
        // CHECK_NEAR(probability.col(0).sum(),1.0, 1e-4);
        normalize();
        log_probability = probability.array().log().matrix();

    }
    BeamProbability::BeamProbability(double var, double epsilon )
    {
        probability = Eigen::MatrixXd(256,256);
        for (uint i = 0 ; i < 256u; i++)
        {
            PDF pdf(static_cast<double>(i),var);
            for (uint j = 0 ; j < 256u ; j++)
            {
                probability(i,j) = pdf.at(static_cast<double> (j)) + epsilon;
            }
        }
        normalize();
        log_probability = probability.array().log().matrix();
    }

    BeamMapping BeamProbability::getMapping() const
    {
        BeamMapping result(256);
        CHECK_NEAR(probability.row(0).sum(),1.0,1e-4);
        CHECK_EQ(probability.rows(),256);
        for (uint i = 0 ; i < 256u ; i++)
        {
            auto val = probability.col(i).maxCoeff(&result(i));
            if (val == 0) result(i) = i ;
            // LOG(INFO) << " Most likely ground truth at measured " << i <<
            //         " is " << result(i) << " at probability " << val;
        }
        return result;
    }
}
