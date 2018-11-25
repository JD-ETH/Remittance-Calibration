#include "remittance_calib/types.hpp"


namespace remittance_calib
{
    BeamProbability::BeamProbability(BeamCounting in)
    {

        probability = in;
        normalize();
        log_probability = probability.array().log().matrix();

    }
    BeamProbability::BeamProbability(int size, double var, double epsilon )
    {
        probability = Eigen::MatrixXd(size,size);
        for (int i = 0 ; i < size; i++)
        {
            PDF pdf(static_cast<double>(i),var);
            for (int j = 0 ; j < size ; j++)
            {
                probability(i,j) = pdf.at(static_cast<double> (j)) + epsilon;
            }
        }
        normalize();
        log_probability = probability.array().log().matrix();
    }

    BeamMapping BeamProbability::getMapping() const
    {
        BeamMapping result(probability.rows());
        CHECK_NEAR(probability.row(0).sum(),1.0,1e-4);
        for (uint i = 0 ; i < probability.rows() ; i++)
        {
            auto val = probability.col(i).maxCoeff(&result(i));
            if (val == 0) result(i) = i ;
            // LOG(INFO) << " Most likely ground truth at measured " << i <<
            //         " is " << result(i) << " at probability " << val;
        }
        return result;
    }
}
