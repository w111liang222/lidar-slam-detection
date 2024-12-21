#ifndef ROBUST_KERNEL_DCS2_HPP
#define ROBUST_KERNEL_DCS2_HPP

#include <g2o/core/g2o_core_api.h>
#include <g2o/core/robust_kernel.h>

namespace g2o {

class RobustKernel;

class G2O_CORE_API RobustKernelDCS2 : public RobustKernel {
 public:
  virtual void robustify(double e2, Vector3& rho) const {
    const double& phi = _delta;
    double scale = (2.0 * phi) / (phi + e2);
    if (scale >= 1.0) {  // limit scale to max of 1 and return this
      rho[0] = e2;
      rho[1] = 1.;
      rho[2] = 0;
    } else {
      double phi_sqr = phi * phi;
      rho[0] = scale * e2 * scale;
      rho[1] = (4 * phi_sqr * (phi - e2)) / std::pow(phi + e2, 3);
      rho[2] = -(8 * phi_sqr * (2 * phi - e2)) / std::pow(phi + e2, 4);
    }
  };
};

}  // namespace g2o

#endif  // ROBUST_KERNEL_DCS2_HPP
