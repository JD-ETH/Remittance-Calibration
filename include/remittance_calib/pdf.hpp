#ifndef REMITTANCE_CALIB_PDf_HPP_
#define REMITTANCE_CALIB_PDf_HPP_

namespace remittance_calib
{
    struct PDF
    {
        PDF():mu(0.0),var(1.0) {}
        PDF(const double & mu, const double & var): mu(mu),var(var){}
        // Raw density
        double at(const double & x)
        {
            return std::exp(-0.5 * (x-mu) * (x-mu) / var / var);
        }
        double mu,var;
    };
}
#endif
