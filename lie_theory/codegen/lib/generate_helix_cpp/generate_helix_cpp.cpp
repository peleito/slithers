//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: generate_helix_cpp.cpp
//
// MATLAB Coder version            : 5.5
// C/C++ source code generated on  : 16-Feb-2023 15:04:13
//

// Include Files
#include "generate_helix_cpp.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <cmath>

// Function Definitions
//
// generate_helix Summary of this function goes here
//    Detailed explanation goes here
//
// Arguments    : double a
//                double b
//                double dt
//                double b_time
//                coder::array<double, 3U> &poses
// Return Type  : void
//
void generate_helix_cpp(double a, double b, double dt, double b_time,
                        coder::array<double, 3U> &poses)
{
  coder::array<double, 3U> rotation_mat;
  coder::array<double, 2U> b_a;
  coder::array<double, 2U> b_b;
  coder::array<double, 2U> c_b;
  coder::array<double, 2U> rotation_vecs;
  coder::array<double, 2U> t;
  coder::array<double, 2U> y;
  double apnd;
  double cdiff;
  double ndbl;
  int i;
  int nm1d2;
  int nx;
  if (std::isnan(dt) || std::isnan(b_time)) {
    t.set_size(1, 1);
    t[0] = rtNaN;
  } else if ((dt == 0.0) || ((b_time > 0.0) && (dt < 0.0)) ||
             ((b_time < 0.0) && (dt > 0.0))) {
    t.set_size(1, 0);
  } else if (std::isinf(b_time) && std::isinf(dt)) {
    t.set_size(1, 1);
    t[0] = rtNaN;
  } else if (std::isinf(dt)) {
    t.set_size(1, 1);
    t[0] = 0.0;
  } else if (std::floor(dt) == dt) {
    nx = static_cast<int>(b_time / dt);
    t.set_size(1, nx + 1);
    for (i = 0; i <= nx; i++) {
      t[i] = dt * static_cast<double>(i);
    }
  } else {
    ndbl = std::floor(b_time / dt + 0.5);
    apnd = ndbl * dt;
    if (dt > 0.0) {
      cdiff = apnd - b_time;
    } else {
      cdiff = b_time - apnd;
    }
    if (std::abs(cdiff) < 4.4408920985006262E-16 * std::abs(b_time)) {
      ndbl++;
      apnd = b_time;
    } else if (cdiff > 0.0) {
      apnd = (ndbl - 1.0) * dt;
    } else {
      ndbl++;
    }
    if (ndbl >= 0.0) {
      nx = static_cast<int>(ndbl);
    } else {
      nx = 0;
    }
    t.set_size(1, nx);
    if (nx > 0) {
      t[0] = 0.0;
      if (nx > 1) {
        t[nx - 1] = apnd;
        nm1d2 = (nx - 1) / 2;
        for (int k{0}; k <= nm1d2 - 2; k++) {
          ndbl = (static_cast<double>(k) + 1.0) * dt;
          t[k + 1] = ndbl;
          t[(nx - k) - 2] = apnd - ndbl;
        }
        if (nm1d2 << 1 == nx - 1) {
          t[nm1d2] = apnd / 2.0;
        } else {
          ndbl = static_cast<double>(nm1d2) * dt;
          t[nm1d2] = ndbl;
          t[nm1d2 + 1] = apnd - ndbl;
        }
      }
    }
  }
  b_b.set_size(1, t.size(1));
  nx = t.size(1);
  for (i = 0; i < nx; i++) {
    b_b[i] = t[i];
  }
  nx = t.size(1);
  for (int k{0}; k < nx; k++) {
    b_b[k] = std::cos(b_b[k]);
  }
  c_b.set_size(1, t.size(1));
  nx = t.size(1);
  for (i = 0; i < nx; i++) {
    c_b[i] = t[i];
  }
  nx = t.size(1);
  for (int k{0}; k < nx; k++) {
    c_b[k] = std::sin(c_b[k]);
  }
  y.set_size(1, t.size(1));
  nx = t.size(1);
  for (i = 0; i < nx; i++) {
    y[i] = 1.5707963267948966 * t[i];
  }
  nx = y.size(1);
  for (int k{0}; k < nx; k++) {
    y[k] = std::cos(y[k]);
  }
  rotation_vecs.set_size(t.size(1), 3);
  nx = t.size(1);
  for (i = 0; i < nx; i++) {
    rotation_vecs[i] = 0.0;
  }
  nx = t.size(1);
  for (i = 0; i < nx; i++) {
    rotation_vecs[i + rotation_vecs.size(0)] = 0.0;
  }
  nx = t.size(1);
  for (i = 0; i < nx; i++) {
    rotation_vecs[i + rotation_vecs.size(0) * 2] = t[i] + 3.1415926535897931;
  }
  rotation_mat.set_size(3, 3, t.size(1));
  nx = 9 * t.size(1);
  for (i = 0; i < nx; i++) {
    rotation_mat[i] = 0.0;
  }
  i = t.size(1);
  for (int b_i{0}; b_i < i; b_i++) {
    double theta;
    ndbl = 3.3121686421112381E-170;
    cdiff = std::abs(rotation_vecs[b_i]);
    if (cdiff > 3.3121686421112381E-170) {
      theta = 1.0;
      ndbl = cdiff;
    } else {
      apnd = cdiff / 3.3121686421112381E-170;
      theta = apnd * apnd;
    }
    cdiff = std::abs(rotation_vecs[b_i + rotation_vecs.size(0)]);
    if (cdiff > ndbl) {
      apnd = ndbl / cdiff;
      theta = theta * apnd * apnd + 1.0;
      ndbl = cdiff;
    } else {
      apnd = cdiff / ndbl;
      theta += apnd * apnd;
    }
    cdiff = std::abs(rotation_vecs[b_i + rotation_vecs.size(0) * 2]);
    if (cdiff > ndbl) {
      apnd = ndbl / cdiff;
      theta = theta * apnd * apnd + 1.0;
      ndbl = cdiff;
    } else {
      apnd = cdiff / ndbl;
      theta += apnd * apnd;
    }
    theta = ndbl * std::sqrt(theta);
    if (theta < 1.0E-6) {
      for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
        rotation_mat[3 * nm1d2 + 9 * b_i] = 0.0;
        rotation_mat[(3 * nm1d2 + 9 * b_i) + 1] = 0.0;
        rotation_mat[(3 * nm1d2 + 9 * b_i) + 2] = 0.0;
      }
      rotation_mat[9 * b_i] = 1.0;
      rotation_mat[9 * b_i + 4] = 1.0;
      rotation_mat[9 * b_i + 8] = 1.0;
    } else {
      double d_a[9];
      double u[3];
      double alpha;
      double u_tmp;
      signed char c_a[9];
      cdiff = rotation_vecs[b_i] / theta;
      u[0] = cdiff;
      apnd = rotation_vecs[b_i + rotation_vecs.size(0)] / theta;
      u[1] = apnd;
      u_tmp = rotation_vecs[b_i + rotation_vecs.size(0) * 2] / theta;
      u[2] = u_tmp;
      alpha = std::cos(theta);
      ndbl = std::sin(theta);
      for (nm1d2 = 0; nm1d2 < 9; nm1d2++) {
        c_a[nm1d2] = 0;
      }
      d_a[0] = ndbl * 0.0;
      d_a[3] = ndbl * -u_tmp;
      d_a[6] = ndbl * apnd;
      d_a[1] = ndbl * u_tmp;
      d_a[4] = ndbl * 0.0;
      d_a[7] = ndbl * -cdiff;
      d_a[2] = ndbl * -apnd;
      d_a[5] = ndbl * cdiff;
      d_a[8] = ndbl * 0.0;
      for (int k{0}; k < 3; k++) {
        c_a[k + 3 * k] = 1;
        ndbl = u[k];
        rotation_mat[3 * k + 9 * b_i] =
            (static_cast<double>(c_a[3 * k]) * alpha + d_a[3 * k]) +
            (1.0 - alpha) * (cdiff * ndbl);
        nx = 3 * k + 1;
        rotation_mat[(3 * k + 9 * b_i) + 1] =
            (static_cast<double>(c_a[nx]) * alpha + d_a[nx]) +
            (1.0 - alpha) * (apnd * ndbl);
        nx = 3 * k + 2;
        rotation_mat[(3 * k + 9 * b_i) + 2] =
            (static_cast<double>(c_a[nx]) * alpha + d_a[nx]) +
            (1.0 - alpha) * (u_tmp * ndbl);
      }
    }
  }
  //  plotTransforms(translation,rotation_mat)
  poses.set_size(4, 4, t.size(1));
  nx = t.size(1) << 4;
  for (i = 0; i < nx; i++) {
    poses[i] = 0.0;
  }
  nx = rotation_mat.size(2);
  for (i = 0; i < nx; i++) {
    for (nm1d2 = 0; nm1d2 < 3; nm1d2++) {
      poses[4 * nm1d2 + 16 * i] = rotation_mat[3 * nm1d2 + 9 * i];
      poses[(4 * nm1d2 + 16 * i) + 1] = rotation_mat[(3 * nm1d2 + 9 * i) + 1];
      poses[(4 * nm1d2 + 16 * i) + 2] = rotation_mat[(3 * nm1d2 + 9 * i) + 2];
    }
  }
  b_a.set_size(3, b_b.size(1));
  nx = b_b.size(1);
  for (i = 0; i < nx; i++) {
    b_a[3 * i] = a * b_b[i];
  }
  nx = c_b.size(1);
  for (i = 0; i < nx; i++) {
    b_a[3 * i + 1] = a * c_b[i];
  }
  nx = y.size(1);
  for (i = 0; i < nx; i++) {
    b_a[3 * i + 2] = b * y[i];
  }
  nx = t.size(1);
  for (i = 0; i < nx; i++) {
    poses[16 * i + 12] = b_a[3 * i];
    poses[16 * i + 13] = b_a[3 * i + 1];
    poses[16 * i + 14] = b_a[3 * i + 2];
  }
  nx = poses.size(2);
  for (i = 0; i < nx; i++) {
    poses[16 * i + 15] = 1.0;
  }
  //  plotTransforms(poses)
}

//
// File trailer for generate_helix_cpp.cpp
//
// [EOF]
//
