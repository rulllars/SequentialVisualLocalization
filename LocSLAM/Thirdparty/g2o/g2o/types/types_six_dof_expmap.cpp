// g2o - General Graph Optimization
// Copyright (C) 2011 H. Strasdat
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

#include "types_six_dof_expmap.h"

#include "../core/factory.h"
#include "../stuff/macros.h"

namespace g2o
{

using namespace std;

Vector2d project2d(const Vector3d &v)
{
  Vector2d res;
  res(0) = v(0) / v(2);
  res(1) = v(1) / v(2);
  return res;
}

Vector3d unproject2d(const Vector2d &v)
{
  Vector3d res;
  res(0) = v(0);
  res(1) = v(1);
  res(2) = 1;
  return res;
}

// **************************************************************
// VertexSE3Expmap
// **************************************************************

VertexSE3Expmap::VertexSE3Expmap() : BaseVertex<6, SE3Quat>()
{
}

bool VertexSE3Expmap::read(std::istream &is)
{
  Vector7d est;
  for (int i = 0; i < 7; i++)
    is >> est[i];
  SE3Quat cam2world;
  cam2world.fromVector(est);
  setEstimate(cam2world.inverse());
  return true;
}

bool VertexSE3Expmap::write(std::ostream &os) const
{
  SE3Quat cam2world(estimate().inverse());
  for (int i = 0; i < 7; i++)
    os << cam2world[i] << " ";
  return os.good();
}

// **************************************************************
// EdgeSE3ProjectXYZ
// **************************************************************

EdgeSE3ProjectXYZ::EdgeSE3ProjectXYZ() : BaseBinaryEdge<2, Vector2d, VertexSBAPointXYZ, VertexSE3Expmap>()
{
}

bool EdgeSE3ProjectXYZ::read(std::istream &is)
{
  for (int i = 0; i < 2; i++)
  {
    is >> _measurement[i];
  }
  for (int i = 0; i < 2; i++)
    for (int j = i; j < 2; j++)
    {
      is >> information()(i, j);
      if (i != j)
        information()(j, i) = information()(i, j);
    }
  return true;
}

bool EdgeSE3ProjectXYZ::write(std::ostream &os) const
{

  for (int i = 0; i < 2; i++)
  {
    os << measurement()[i] << " ";
  }

  for (int i = 0; i < 2; i++)
    for (int j = i; j < 2; j++)
    {
      os << " " << information()(i, j);
    }
  return os.good();
}

/*
void EdgeSE3ProjectXYZ::linearizeOplus()
{
  VertexSE3Expmap *vj = static_cast<VertexSE3Expmap *>(_vertices[1]);
  SE3Quat Tbw(vj->estimate());
  VertexSBAPointXYZ *vi = static_cast<VertexSBAPointXYZ *>(_vertices[0]);
  Vector3d Xw = vi->estimate();
  Vector3d Xc = Tcb.map(Tbw.map(Xw));

  double x = Xc[0];
  double y = Xc[1];
  double z = Xc[2];
  double z_2 = z * z;

  // Jacobian of camera projection with coordinate normalization
  Matrix<double, 2, 3> tmp;
  tmp(0, 0) = fx;
  tmp(0, 1) = 0;
  tmp(0, 2) = -x / z * fx;

  tmp(1, 0) = 0;
  tmp(1, 1) = fy;
  tmp(1, 2) = -y / z * fy;

  Matrix<double, 2, 3> J_camProj = 1. / z * tmp;

  // error Jacobian w.r.t map point: - Jacobian of camera projection * Rcb * Rbw
  _jacobianOplusXi = -J_camProj * Tcb.rotation().toRotationMatrix() * Tbw.rotation().toRotationMatrix();

  // error Jacobian w.r.t. body position: Jacobian of camera projection * Rcb
  Matrix<double, 2, 3> J_tbw = -J_camProj * Tcb.rotation().toRotationMatrix();

  // error Jacobian w.r.t. body rotation: Jacobian of camera projection * - Rcb * [Rbw * Xw]^ (see, e.g. ethaneade.com/lie.pdf)
  Vector3d Xb = Tbw.rotation().toRotationMatrix() * Xw;
  Matrix<double, 3, 3> Xb_hat;
  Xb_hat << 0, -Xb(2), Xb(1),
      Xb(2), 0, -Xb(0),
      -Xb(1), Xb(0), 0;

  Matrix<double, 2, 3> J_Rbw = -J_camProj * -Tcb.rotation().toRotationMatrix() * Xb_hat;

  /*
  // error Jacobian w.r.t. body rotation: Jacobian of camera projection * - Rcb * [Rbw * Xw]^ (see, e.g. a micro lie theory for satate estimation in robotics)
  Matrix<double, 3, 3> Xw_hat;
  Xw_hat << 0, -Xw(2), Xw(1),
      Xw(2), 0, -Xw(0),
      -Xw(1), Xw(0), 0;

  Matrix<double, 2, 3> J_Rbw = -J_camProj * -Tcb.rotation().toRotationMatrix() * Tbw.rotation().toRotationMatrix() * Xw_hat;
*/

//_jacobianOplusXi.block<2, 3>(0, 0) = J_Rbw;
// _jacobianOplusXi.block<2, 3>(0, 3) = J_tbw;

/*
  _jacobianOplusXj(0,0) =  x*y/z_2 *fx;
  _jacobianOplusXj(0,1) = -(1+(x*x/z_2)) *fx;
  _jacobianOplusXj(0,2) = y/z *fx;
  _jacobianOplusXj(0,3) = -1./z *fx;
  _jacobianOplusXj(0,4) = 0;
  _jacobianOplusXj(0,5) = x/z_2 *fx;

  _jacobianOplusXj(1,0) = (1+y*y/z_2) *fy;
  _jacobianOplusXj(1,1) = -x*y/z_2 *fy;
  _jacobianOplusXj(1,2) = -x/z *fy;
  _jacobianOplusXj(1,3) = 0;
  _jacobianOplusXj(1,4) = -1./z *fy;
  _jacobianOplusXj(1,5) = y/z_2 *fy;
  */

//} // namespace g2o

Vector2d EdgeSE3ProjectXYZ::cam_project(const Vector3d &trans_xyz) const
{
  Vector2d proj = project2d(trans_xyz);
  Vector2d res;
  res[0] = proj[0] * fx + cx;
  res[1] = proj[1] * fy + cy;
  return res;
}

// **************************************************************
// EdgeSE3ProjectXYZFixed
// **************************************************************

EdgeSE3ProjectXYZFixed::EdgeSE3ProjectXYZFixed() : BaseBinaryEdge<2, Vector2d, VertexSE3Expmap, VertexSE3Expmap>()
{
}

bool EdgeSE3ProjectXYZFixed::read(std::istream &is)
{
  for (int i = 0; i < 2; i++)
  {
    is >> _measurement[i];
  }
  for (int i = 0; i < 2; i++)
    for (int j = i; j < 2; j++)
    {
      is >> information()(i, j);
      if (i != j)
        information()(j, i) = information()(i, j);
    }
  return true;
}

bool EdgeSE3ProjectXYZFixed::write(std::ostream &os) const
{

  for (int i = 0; i < 2; i++)
  {
    os << measurement()[i] << " ";
  }

  for (int i = 0; i < 2; i++)
    for (int j = i; j < 2; j++)
    {
      os << " " << information()(i, j);
    }
  return os.good();
}

Vector2d EdgeSE3ProjectXYZFixed::cam_project(const Vector3d &trans_xyz) const
{
  Vector2d proj = project2d(trans_xyz);
  Vector2d res;
  res[0] = proj[0] * fx + cx;
  res[1] = proj[1] * fy + cy;
  return res;
}

// **************************************************************
// EdgeSE3ProjectXYZCalib
// **************************************************************

EdgeSE3ProjectXYZCalib::EdgeSE3ProjectXYZCalib() : BaseMultiEdge<2, Vector2d>()
{
  resize(3);
}

Vector2d EdgeSE3ProjectXYZCalib::cam_project(const Vector3d &trans_xyz) const
{
  Vector2d proj = project2d(trans_xyz);
  Vector2d res;
  res[0] = proj[0] * fx + cx;
  res[1] = proj[1] * fy + cy;
  return res;
}

bool EdgeSE3ProjectXYZCalib::read(std::istream &is)
{
}

bool EdgeSE3ProjectXYZCalib::write(std::ostream &os) const
{
}

// **************************************************************
// EdgeStereoSE3ProjectXYZ
// **************************************************************

Vector3d EdgeStereoSE3ProjectXYZ::cam_project(const Vector3d &trans_xyz, const float &bf) const
{
  const float invz = 1.0f / trans_xyz[2];
  Vector3d res;
  res[0] = trans_xyz[0] * invz * fx + cx;
  res[1] = trans_xyz[1] * invz * fy + cy;
  res[2] = res[0] - bf * invz;
  return res;
}

EdgeStereoSE3ProjectXYZ::EdgeStereoSE3ProjectXYZ() : BaseBinaryEdge<3, Vector3d, VertexSBAPointXYZ, VertexSE3Expmap>()
{
}

bool EdgeStereoSE3ProjectXYZ::read(std::istream &is)
{
  for (int i = 0; i <= 3; i++)
  {
    is >> _measurement[i];
  }
  for (int i = 0; i <= 2; i++)
    for (int j = i; j <= 2; j++)
    {
      is >> information()(i, j);
      if (i != j)
        information()(j, i) = information()(i, j);
    }
  return true;
}

bool EdgeStereoSE3ProjectXYZ::write(std::ostream &os) const
{

  for (int i = 0; i <= 3; i++)
  {
    os << measurement()[i] << " ";
  }

  for (int i = 0; i <= 2; i++)
    for (int j = i; j <= 2; j++)
    {
      os << " " << information()(i, j);
    }
  return os.good();
}

void EdgeStereoSE3ProjectXYZ::linearizeOplus()
{
  VertexSE3Expmap *vj = static_cast<VertexSE3Expmap *>(_vertices[1]);
  SE3Quat Tbw(vj->estimate());
  VertexSBAPointXYZ *vi = static_cast<VertexSBAPointXYZ *>(_vertices[0]);
  Vector3d xyz = vi->estimate();
  Vector3d xyz_trans = Tcb.map(Tbw.map(xyz));

  const Matrix3d R = Tcb.rotation().toRotationMatrix() * Tbw.rotation().toRotationMatrix();

  double x = xyz_trans[0];
  double y = xyz_trans[1];
  double z = xyz_trans[2];
  double z_2 = z * z;

  _jacobianOplusXi(0, 0) = -fx * R(0, 0) / z + fx * x * R(2, 0) / z_2;
  _jacobianOplusXi(0, 1) = -fx * R(0, 1) / z + fx * x * R(2, 1) / z_2;
  _jacobianOplusXi(0, 2) = -fx * R(0, 2) / z + fx * x * R(2, 2) / z_2;

  _jacobianOplusXi(1, 0) = -fy * R(1, 0) / z + fy * y * R(2, 0) / z_2;
  _jacobianOplusXi(1, 1) = -fy * R(1, 1) / z + fy * y * R(2, 1) / z_2;
  _jacobianOplusXi(1, 2) = -fy * R(1, 2) / z + fy * y * R(2, 2) / z_2;

  _jacobianOplusXi(2, 0) = _jacobianOplusXi(0, 0) - bf * R(2, 0) / z_2;
  _jacobianOplusXi(2, 1) = _jacobianOplusXi(0, 1) - bf * R(2, 1) / z_2;
  _jacobianOplusXi(2, 2) = _jacobianOplusXi(0, 2) - bf * R(2, 2) / z_2;

  _jacobianOplusXj(0, 0) = x * y / z_2 * fx;
  _jacobianOplusXj(0, 1) = -(1 + (x * x / z_2)) * fx;
  _jacobianOplusXj(0, 2) = y / z * fx;
  _jacobianOplusXj(0, 3) = -1. / z * fx;
  _jacobianOplusXj(0, 4) = 0;
  _jacobianOplusXj(0, 5) = x / z_2 * fx;

  _jacobianOplusXj(1, 0) = (1 + y * y / z_2) * fy;
  _jacobianOplusXj(1, 1) = -x * y / z_2 * fy;
  _jacobianOplusXj(1, 2) = -x / z * fy;
  _jacobianOplusXj(1, 3) = 0;
  _jacobianOplusXj(1, 4) = -1. / z * fy;
  _jacobianOplusXj(1, 5) = y / z_2 * fy;

  _jacobianOplusXj(2, 0) = _jacobianOplusXj(0, 0) - bf * y / z_2;
  _jacobianOplusXj(2, 1) = _jacobianOplusXj(0, 1) + bf * x / z_2;
  _jacobianOplusXj(2, 2) = _jacobianOplusXj(0, 2);
  _jacobianOplusXj(2, 3) = _jacobianOplusXj(0, 3);
  _jacobianOplusXj(2, 4) = 0;
  _jacobianOplusXj(2, 5) = _jacobianOplusXj(0, 5) - bf / z_2;
}

// **************************************************************
// EdgeSE3ProjectXYZOnlyPose
// **************************************************************

bool EdgeSE3ProjectXYZOnlyPose::read(std::istream &is)
{
  for (int i = 0; i < 2; i++)
  {
    is >> _measurement[i];
  }
  for (int i = 0; i < 2; i++)
    for (int j = i; j < 2; j++)
    {
      is >> information()(i, j);
      if (i != j)
        information()(j, i) = information()(i, j);
    }
  return true;
}

bool EdgeSE3ProjectXYZOnlyPose::write(std::ostream &os) const
{

  for (int i = 0; i < 2; i++)
  {
    os << measurement()[i] << " ";
  }

  for (int i = 0; i < 2; i++)
    for (int j = i; j < 2; j++)
    {
      os << " " << information()(i, j);
    }
  return os.good();
}
/*
void EdgeSE3ProjectXYZOnlyPose::linearizeOplus()
{
  VertexSE3Expmap *vi = static_cast<VertexSE3Expmap *>(_vertices[0]);
  SE3Quat Tbw = vi->estimate();
  Vector3d xyz_trans = Tcb.map(vi->estimate().map(Xw));

  double x = xyz_trans[0];
  double y = xyz_trans[1];
  double z = xyz_trans[2];
  double invz = 1.0 / z;
  double invz_2 = invz * invz;

  // Jacobian of camera projection with coordinate normalization
  Matrix<double, 2, 3> tmp;
  tmp(0, 0) = fx;
  tmp(0, 1) = 0;
  tmp(0, 2) = -x / z * fx;

  tmp(1, 0) = 0;
  tmp(1, 1) = fy;
  tmp(1, 2) = -y / z * fy;

  Matrix<double, 2, 3> J_camProj = 1. / z * tmp;

  // error Jacobian w.r.t. body position: Jacobian of camera projection * Rcb
  Matrix<double, 2, 3> J_tbw = -J_camProj * Tcb.rotation().toRotationMatrix();

  // error Jacobian w.r.t. body rotation: Jacobian of camera projection * - Rcb * [Rbw * Xw]^ (see, e.g. ethaneade.com/lie.pdf)
  Vector3d Xb = Tbw.rotation().toRotationMatrix() * Xw;
  Matrix<double, 3, 3> Xb_hat;
  Xb_hat << 0, -Xb(2), Xb(1),
      Xb(2), 0, -Xb(0),
      -Xb(1), Xb(0), 0;

  //Matrix<double, 2, 3> J_Rbw = -J_camProj * -Tcb.rotation().toRotationMatrix() * Xb_hat;
  Matrix<double, 2, 3> J_Rbw = -J_camProj * -Tcb.rotation().toRotationMatrix() * Xb_hat;
*/
/*
  // error Jacobian w.r.t. body rotation: Jacobian of camera projection * - Rcb * [Rbw * Xw]^ (see, e.g. a micro lie theory for state estimmation in robotics)
  Matrix<double, 3, 3> Xw_hat;
  Xw_hat << 0, -Xw(2), Xw(1),
      Xw(2), 0, -Xw(0),
      -Xw(1), Xw(0), 0;

  Matrix<double, 2, 3> J_Rbw = -J_camProj * -Tcb.rotation().toRotationMatrix() * Tbw.rotation().toRotationMatrix() * Xw_hat;
*/
/*
  _jacobianOplusXi.block<2, 3>(0, 0) = J_Rbw;
  _jacobianOplusXi.block<2, 3>(0, 3) = J_tbw;
*/
/*
  _jacobianOplusXi(0, 0) = x * y * invz_2 * fx;
  _jacobianOplusXi(0, 1) = -(1 + (x * x * invz_2)) * fx;
  _jacobianOplusXi(0, 2) = y * invz * fx;
  _jacobianOplusXi(0, 3) = -invz * fx;
  _jacobianOplusXi(0, 4) = 0;
  _jacobianOplusXi(0, 5) = x * invz_2 * fx;

  _jacobianOplusXi(1, 0) = (1 + y * y * invz_2) * fy;
  _jacobianOplusXi(1, 1) = -x * y * invz_2 * fy;
  _jacobianOplusXi(1, 2) = -x * invz * fy;
  _jacobianOplusXi(1, 3) = 0;
  _jacobianOplusXi(1, 4) = -invz * fy;
  _jacobianOplusXi(1, 5) = y * invz_2 * fy;
  */
//s}

Vector2d EdgeSE3ProjectXYZOnlyPose::cam_project(const Vector3d &trans_xyz) const
{
  Vector2d proj = project2d(trans_xyz);
  Vector2d res;
  res[0] = proj[0] * fx + cx;
  res[1] = proj[1] * fy + cy;
  return res;
}

// **************************************************************
// EdgeStereoSE3ProjectXYZOnlyPose
// **************************************************************

Vector3d EdgeStereoSE3ProjectXYZOnlyPose::cam_project(const Vector3d &trans_xyz) const
{
  const float invz = 1.0f / trans_xyz[2];
  Vector3d res;
  res[0] = trans_xyz[0] * invz * fx + cx;
  res[1] = trans_xyz[1] * invz * fy + cy;
  res[2] = res[0] - bf * invz;
  return res;
}

bool EdgeStereoSE3ProjectXYZOnlyPose::read(std::istream &is)
{
  for (int i = 0; i <= 3; i++)
  {
    is >> _measurement[i];
  }
  for (int i = 0; i <= 2; i++)
    for (int j = i; j <= 2; j++)
    {
      is >> information()(i, j);
      if (i != j)
        information()(j, i) = information()(i, j);
    }
  return true;
}

bool EdgeStereoSE3ProjectXYZOnlyPose::write(std::ostream &os) const
{

  for (int i = 0; i <= 3; i++)
  {
    os << measurement()[i] << " ";
  }

  for (int i = 0; i <= 2; i++)
    for (int j = i; j <= 2; j++)
    {
      os << " " << information()(i, j);
    }
  return os.good();
}

void EdgeStereoSE3ProjectXYZOnlyPose::linearizeOplus()
{
  VertexSE3Expmap *vi = static_cast<VertexSE3Expmap *>(_vertices[0]);
  Vector3d xyz_trans = Tcb.map(vi->estimate().map(Xw));

  double x = xyz_trans[0];
  double y = xyz_trans[1];
  double invz = 1.0 / xyz_trans[2];
  double invz_2 = invz * invz;

  _jacobianOplusXi(0, 0) = x * y * invz_2 * fx;
  _jacobianOplusXi(0, 1) = -(1 + (x * x * invz_2)) * fx;
  _jacobianOplusXi(0, 2) = y * invz * fx;
  _jacobianOplusXi(0, 3) = -invz * fx;
  _jacobianOplusXi(0, 4) = 0;
  _jacobianOplusXi(0, 5) = x * invz_2 * fx;

  _jacobianOplusXi(1, 0) = (1 + y * y * invz_2) * fy;
  _jacobianOplusXi(1, 1) = -x * y * invz_2 * fy;
  _jacobianOplusXi(1, 2) = -x * invz * fy;
  _jacobianOplusXi(1, 3) = 0;
  _jacobianOplusXi(1, 4) = -invz * fy;
  _jacobianOplusXi(1, 5) = y * invz_2 * fy;

  _jacobianOplusXi(2, 0) = _jacobianOplusXi(0, 0) - bf * y * invz_2;
  _jacobianOplusXi(2, 1) = _jacobianOplusXi(0, 1) + bf * x * invz_2;
  _jacobianOplusXi(2, 2) = _jacobianOplusXi(0, 2);
  _jacobianOplusXi(2, 3) = _jacobianOplusXi(0, 3);
  _jacobianOplusXi(2, 4) = 0;
  _jacobianOplusXi(2, 5) = _jacobianOplusXi(0, 5) - bf * invz_2;
}

// **************************************************************
// EdgeSE3PosePose
// **************************************************************

bool EdgeSE3PosePose::read(std::istream &is)
{
}

bool EdgeSE3PosePose::write(std::ostream &os) const
{
}

// **************************************************************
// EdgeSE3PosePrior
// **************************************************************

bool EdgeSE3PosePrior::read(std::istream &is)
{
}

bool EdgeSE3PosePrior::write(std::ostream &os) const
{
}

} // namespace g2o
