#include "remittance_calib/types.hpp"


namespace remittance_calib
{
    BeamProbability::BeamProbability(double var = 1.0, double epsilon = 0.1)
    {
        for (uint i = 0 ; i < 256u; i++)
        {
            PDF pdf(static_cast<double>(i),var);
            for (uint j = 0 ; j < 256u ; j++)
            {
                probability(i,j) = pdf.at(static_cast<double> (j)) + epsilon;
            }
        }
        normalize();
    }

    BeamMapping BeamProbability::getMapping() const
    {
        BeamMapping result;
        CHECK_EQ(probability.row(0).sum(),1);
        for (uint i = 0 ; i < 256u ; i++)
        {
            auto val = probability.col(i).maxCoeff(result(i));
            LOG(INFO) << " Most likely ground truth at measured " << i <<
                    " is " << result(i) << " at probability " << val;
        }
    }
}
