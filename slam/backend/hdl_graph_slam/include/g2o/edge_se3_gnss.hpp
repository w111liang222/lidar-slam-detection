// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef KKL_G2O_EDGE_SE3_GNSS_HPP
#define KKL_G2O_EDGE_SE3_GNSS_HPP

#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d_addons/types_slam3d_addons.h>

namespace g2o {
class EdgeSE3GNSS : public g2o::BaseBinaryEdge<3, Eigen::Vector3d, g2o::VertexSE3, g2o::VertexPointXYZ> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgeSE3GNSS() : g2o::BaseBinaryEdge<3, Eigen::Vector3d, g2o::VertexSE3, g2o::VertexPointXYZ>() {}

  void computeError() override {
    const g2o::VertexSE3* v1 = static_cast<const g2o::VertexSE3*>(_vertices[0]);
    const g2o::VertexPointXYZ* v2 = static_cast<const g2o::VertexPointXYZ*>(_vertices[1]);

    Eigen::Isometry3d estimate = v1->estimate();
    _error = estimate.translation() - (_measurement + estimate.linear() * v2->estimate());
  }

  void setMeasurement(const Eigen::Vector3d& m) override {
    _measurement = m;
  }

  virtual bool read(std::istream& is) override {
    Eigen::Vector3d v;
    is >> v(0) >> v(1) >> v(2);
    setMeasurement(v);
    for(int i = 0; i < information().rows(); ++i)
      for(int j = i; j < information().cols(); ++j) {
        is >> information()(i, j);
        if(i != j) information()(j, i) = information()(i, j);
      }
    return true;
  }
  virtual bool write(std::ostream& os) const override {
    Eigen::Vector3d v = _measurement;
    os << v(0) << " " << v(1) << " " << v(2) << " ";
    for(int i = 0; i < information().rows(); ++i)
      for(int j = i; j < information().cols(); ++j) os << " " << information()(i, j);
    return os.good();
  }
};
}  // namespace g2o

#endif
