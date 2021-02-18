//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: IMU_Fusion.cpp
//
// Code generated for Simulink model 'IMU_Fusion'.
//
// Model version                  : 1.3
// Simulink Coder version         : 9.4 (R2020b) 29-Jul-2020
// C/C++ source code generated on : Thu Feb 18 00:20:09 2021
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM Cortex-M
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include "IMU_Fusion.h"
#include "IMU_Fusion_private.h"

void IMU_FusionModelClass::IMUFusionCommon_computeAngularV(const real_T gfast[3],
  const real_T offset[3], real_T av[3])
{
  av[0] = gfast[0] - offset[0];
  av[1] = gfast[1] - offset[1];
  av[2] = gfast[2] - offset[2];
}

void IMU_FusionModelClass::IMU_Fusion_NED_ecompass(const real_T a[3], const
  real_T m[3], real_T R[9])
{
  real_T x[9];
  real_T Reast[3];
  real_T R_0;
  int32_T iy;
  int32_T xpageoffset;
  boolean_T b[9];
  boolean_T y[3];
  boolean_T exitg1;
  boolean_T nanPageIdx;
  Reast[0] = a[1] * m[2] - a[2] * m[1];
  Reast[1] = a[2] * m[0] - a[0] * m[2];
  Reast[2] = a[0] * m[1] - a[1] * m[0];
  R[6] = a[0];
  R[3] = Reast[0];
  R[7] = a[1];
  R[4] = Reast[1];
  R[8] = a[2];
  R[5] = Reast[2];
  R[0] = Reast[1] * a[2] - Reast[2] * a[1];
  R[1] = Reast[2] * a[0] - Reast[0] * a[2];
  R[2] = Reast[0] * a[1] - Reast[1] * a[0];
  for (iy = 0; iy < 9; iy++) {
    R_0 = R[iy];
    x[iy] = R_0 * R_0;
  }

  for (iy = 0; iy < 3; iy++) {
    xpageoffset = iy * 3;
    Reast[iy] = x[xpageoffset + 2] + (x[xpageoffset + 1] + x[xpageoffset]);
  }

  Reast[0] = std::sqrt(Reast[0]);
  Reast[1] = std::sqrt(Reast[1]);
  Reast[2] = std::sqrt(Reast[2]);
  std::memcpy(&x[0], &R[0], 9U * sizeof(real_T));
  iy = 0;
  for (xpageoffset = 0; xpageoffset < 3; xpageoffset++) {
    R[iy] = x[iy] / Reast[xpageoffset];
    R[iy + 1] = x[iy + 1] / Reast[xpageoffset];
    R[iy + 2] = x[iy + 2] / Reast[xpageoffset];
    iy += 3;
  }

  for (iy = 0; iy < 9; iy++) {
    b[iy] = rtIsNaN(R[iy]);
  }

  y[0] = false;
  y[1] = false;
  y[2] = false;
  xpageoffset = 1;
  exitg1 = false;
  while ((!exitg1) && (xpageoffset <= 3)) {
    if (!b[xpageoffset - 1]) {
      xpageoffset++;
    } else {
      y[0] = true;
      exitg1 = true;
    }
  }

  xpageoffset = 4;
  exitg1 = false;
  while ((!exitg1) && (xpageoffset <= 6)) {
    if (!b[xpageoffset - 1]) {
      xpageoffset++;
    } else {
      y[1] = true;
      exitg1 = true;
    }
  }

  xpageoffset = 7;
  exitg1 = false;
  while ((!exitg1) && (xpageoffset <= 9)) {
    if (!b[xpageoffset - 1]) {
      xpageoffset++;
    } else {
      y[2] = true;
      exitg1 = true;
    }
  }

  nanPageIdx = false;
  iy = 0;
  exitg1 = false;
  while ((!exitg1) && (iy < 3)) {
    if (!y[iy]) {
      iy++;
    } else {
      nanPageIdx = true;
      exitg1 = true;
    }
  }

  if (nanPageIdx) {
    std::memset(&R[0], 0, 9U * sizeof(real_T));
    R[0] = 1.0;
    R[4] = 1.0;
    R[8] = 1.0;
  }
}

void IMU_FusionModelClass::IMU_Fusio_quaternion_quaternion(const real_T
  varargin_1[9], real_T *obj_a, real_T *obj_b, real_T *obj_c, real_T *obj_d)
{
  real_T psquared[4];
  real_T tmp;
  real_T tr;
  int32_T b;
  int32_T b_idx;
  int32_T b_k;
  boolean_T exitg1;
  tr = (varargin_1[0] + varargin_1[4]) + varargin_1[8];
  psquared[0] = (tr * 2.0 + 1.0) - tr;
  psquared[1] = (2.0 * varargin_1[0] + 1.0) - tr;
  psquared[2] = (2.0 * varargin_1[4] + 1.0) - tr;
  psquared[3] = (2.0 * varargin_1[8] + 1.0) - tr;
  if (!rtIsNaN(psquared[0])) {
    b_idx = 1;
  } else {
    b_idx = 0;
    b_k = 2;
    exitg1 = false;
    while ((!exitg1) && (b_k < 5)) {
      if (!rtIsNaN(psquared[b_k - 1])) {
        b_idx = b_k;
        exitg1 = true;
      } else {
        b_k++;
      }
    }
  }

  if (b_idx == 0) {
    tr = psquared[0];
    b_idx = 1;
  } else {
    tr = psquared[b_idx - 1];
    b = b_idx;
    for (b_k = b + 1; b_k < 5; b_k++) {
      tmp = psquared[b_k - 1];
      if (tr < tmp) {
        tr = tmp;
        b_idx = b_k;
      }
    }
  }

  switch (b_idx) {
   case 1:
    tr = std::sqrt(tr);
    *obj_a = 0.5 * tr;
    tr = 0.5 / tr;
    *obj_b = (varargin_1[7] - varargin_1[5]) * tr;
    *obj_c = (varargin_1[2] - varargin_1[6]) * tr;
    *obj_d = (varargin_1[3] - varargin_1[1]) * tr;
    break;

   case 2:
    tr = std::sqrt(tr);
    *obj_b = 0.5 * tr;
    tr = 0.5 / tr;
    *obj_a = (varargin_1[7] - varargin_1[5]) * tr;
    *obj_c = (varargin_1[3] + varargin_1[1]) * tr;
    *obj_d = (varargin_1[2] + varargin_1[6]) * tr;
    break;

   case 3:
    tr = std::sqrt(tr);
    *obj_c = 0.5 * tr;
    tr = 0.5 / tr;
    *obj_a = (varargin_1[2] - varargin_1[6]) * tr;
    *obj_b = (varargin_1[3] + varargin_1[1]) * tr;
    *obj_d = (varargin_1[7] + varargin_1[5]) * tr;
    break;

   default:
    tr = std::sqrt(tr);
    *obj_d = 0.5 * tr;
    tr = 0.5 / tr;
    *obj_a = (varargin_1[3] - varargin_1[1]) * tr;
    *obj_b = (varargin_1[2] + varargin_1[6]) * tr;
    *obj_c = (varargin_1[7] + varargin_1[5]) * tr;
    break;
  }

  if (*obj_a < 0.0) {
    *obj_a = -*obj_a;
    *obj_b = -*obj_b;
    *obj_c = -*obj_c;
    *obj_d = -*obj_d;
  }
}

void IMU_FusionModelClass::IMU_Fus_quaternion_quaternion_o(const real_T
  varargin_1[3], real_T *obj_a, real_T *obj_b, real_T *obj_c, real_T *obj_d)
{
  real_T st;
  real_T theta;
  *obj_a = 1.0;
  *obj_b = 0.0;
  *obj_c = 0.0;
  *obj_d = 0.0;
  theta = std::sqrt((varargin_1[0] * varargin_1[0] + varargin_1[1] * varargin_1
                     [1]) + varargin_1[2] * varargin_1[2]);
  st = std::sin(theta / 2.0);
  if (theta != 0.0) {
    *obj_a = std::cos(theta / 2.0);
    *obj_b = varargin_1[0] / theta * st;
    *obj_c = varargin_1[1] / theta * st;
    *obj_d = varargin_1[2] / theta * st;
  }
}

void IMU_FusionModelClass::IMUFusionCommon_predictOrientat(const
  fusion_simulink_ahrsfilter_IM_T *obj, const real_T gfast[3], const real_T
  offset[3], real_T qorient_a, real_T qorient_b, real_T qorient_c, real_T
  qorient_d, real_T *b_qorient_a, real_T *b_qorient_b, real_T *b_qorient_c,
  real_T *b_qorient_d)
{
  real_T c[3];
  real_T deltaq_a;
  real_T deltaq_b;
  real_T deltaq_c;
  real_T deltaq_d;
  c[0] = (gfast[0] - offset[0]) * obj->pSensorPeriod;
  c[1] = (gfast[1] - offset[1]) * obj->pSensorPeriod;
  c[2] = (gfast[2] - offset[2]) * obj->pSensorPeriod;
  IMU_Fus_quaternion_quaternion_o(c, &deltaq_a, &deltaq_b, &deltaq_c, &deltaq_d);
  *b_qorient_a = ((qorient_a * deltaq_a - qorient_b * deltaq_b) - qorient_c *
                  deltaq_c) - qorient_d * deltaq_d;
  *b_qorient_b = ((qorient_a * deltaq_b + qorient_b * deltaq_a) + qorient_c *
                  deltaq_d) - qorient_d * deltaq_c;
  *b_qorient_c = ((qorient_a * deltaq_c - qorient_b * deltaq_d) + qorient_c *
                  deltaq_a) + qorient_d * deltaq_b;
  *b_qorient_d = ((qorient_a * deltaq_d + qorient_b * deltaq_c) - qorient_c *
                  deltaq_b) + qorient_d * deltaq_a;
  if (*b_qorient_a < 0.0) {
    *b_qorient_a = -*b_qorient_a;
    *b_qorient_b = -*b_qorient_b;
    *b_qorient_c = -*b_qorient_c;
    *b_qorient_d = -*b_qorient_d;
  }
}

void IMU_FusionModelClass::IMU_Fusio_quaternionBase_rotmat(real_T q_a, real_T
  q_b, real_T q_c, real_T q_d, real_T r[9])
{
  real_T aasq;
  real_T ac2;
  real_T ad2;
  real_T bc2;
  real_T bd2;
  real_T cd2;
  real_T n;
  n = std::sqrt(((q_a * q_a + q_b * q_b) + q_c * q_c) + q_d * q_d);
  q_a /= n;
  q_b /= n;
  q_c /= n;
  q_d /= n;
  n = q_a * q_b * 2.0;
  ac2 = q_a * q_c * 2.0;
  ad2 = q_a * q_d * 2.0;
  bc2 = q_b * q_c * 2.0;
  bd2 = q_b * q_d * 2.0;
  cd2 = q_c * q_d * 2.0;
  aasq = q_a * q_a * 2.0 - 1.0;
  r[0] = q_b * q_b * 2.0 + aasq;
  r[3] = bc2 + ad2;
  r[6] = bd2 - ac2;
  r[1] = bc2 - ad2;
  r[4] = q_c * q_c * 2.0 + aasq;
  r[7] = cd2 + n;
  r[2] = bd2 + ac2;
  r[5] = cd2 - n;
  r[8] = q_d * q_d * 2.0 + aasq;
}

void IMU_FusionModelClass::IMU_Fusion_mrdiv(const real_T A[72], const real_T B
  [36], real_T Y[72])
{
  real_T c_A[36];
  real_T s;
  real_T smax;
  int32_T c;
  int32_T ix;
  int32_T iy;
  int32_T jA;
  int32_T jBcol;
  int32_T jj;
  int32_T jp;
  int32_T jp1j;
  int32_T kBcol;
  int8_T b_ipiv[6];
  int8_T b_ipiv_0;
  std::memcpy(&c_A[0], &B[0], 36U * sizeof(real_T));
  for (c = 0; c < 6; c++) {
    b_ipiv[c] = static_cast<int8_T>(c + 1);
  }

  for (jp = 0; jp < 5; jp++) {
    jBcol = jp * 7 + 2;
    jj = jp * 7;
    c = 6 - jp;
    iy = 1;
    ix = jBcol - 2;
    smax = std::abs(c_A[jj]);
    for (jA = 2; jA <= c; jA++) {
      ix++;
      s = std::abs(c_A[ix]);
      if (s > smax) {
        iy = jA;
        smax = s;
      }
    }

    if (c_A[(jBcol + iy) - 3] != 0.0) {
      if (iy - 1 != 0) {
        b_ipiv[jp] = static_cast<int8_T>(jp + iy);
        ix = jp;
        iy = (jp + iy) - 1;
        for (jA = 0; jA < 6; jA++) {
          smax = c_A[ix];
          c_A[ix] = c_A[iy];
          c_A[iy] = smax;
          ix += 6;
          iy += 6;
        }
      }

      iy = jBcol - jp;
      for (ix = jBcol; ix <= iy + 4; ix++) {
        c_A[ix - 1] /= c_A[jj];
      }
    }

    c = 4 - jp;
    jA = jj;
    jj += 6;
    for (jp1j = 0; jp1j <= c; jp1j++) {
      smax = c_A[jj];
      if (c_A[jj] != 0.0) {
        ix = jBcol - 1;
        kBcol = jA - jp;
        for (iy = jA + 8; iy <= kBcol + 12; iy++) {
          c_A[iy - 1] += c_A[ix] * -smax;
          ix++;
        }
      }

      jj += 6;
      jA += 6;
    }
  }

  std::memcpy(&Y[0], &A[0], 72U * sizeof(real_T));
  for (jp = 0; jp < 6; jp++) {
    jBcol = 12 * jp - 1;
    jj = 6 * jp - 1;
    iy = jp - 1;
    for (jp1j = 0; jp1j <= iy; jp1j++) {
      kBcol = 12 * jp1j - 1;
      if (c_A[(jp1j + jj) + 1] != 0.0) {
        for (c = 0; c < 12; c++) {
          ix = (c + jBcol) + 1;
          Y[ix] -= c_A[(jp1j + jj) + 1] * Y[(c + kBcol) + 1];
        }
      }
    }

    smax = 1.0 / c_A[(jp + jj) + 1];
    for (jp1j = 0; jp1j < 12; jp1j++) {
      ix = (jp1j + jBcol) + 1;
      Y[ix] *= smax;
    }
  }

  for (jp1j = 5; jp1j >= 0; jp1j--) {
    jBcol = 12 * jp1j - 1;
    jj = 6 * jp1j - 1;
    for (jA = jp1j + 2; jA < 7; jA++) {
      kBcol = (jA - 1) * 12 - 1;
      if (c_A[jA + jj] != 0.0) {
        for (c = 0; c < 12; c++) {
          ix = (c + jBcol) + 1;
          Y[ix] -= Y[(c + kBcol) + 1] * c_A[jA + jj];
        }
      }
    }
  }

  for (jp1j = 4; jp1j >= 0; jp1j--) {
    b_ipiv_0 = b_ipiv[jp1j];
    if (jp1j + 1 != b_ipiv_0) {
      for (c = 0; c < 12; c++) {
        jp = 12 * jp1j + c;
        smax = Y[jp];
        ix = 12 * (b_ipiv_0 - 1) + c;
        Y[jp] = Y[ix];
        Y[ix] = smax;
      }
    }
  }
}

real_T rt_atan2d_snf(real_T u0, real_T u1)
{
  real_T y;
  int32_T u0_0;
  int32_T u1_0;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    if (u0 > 0.0) {
      u0_0 = 1;
    } else {
      u0_0 = -1;
    }

    if (u1 > 0.0) {
      u1_0 = 1;
    } else {
      u1_0 = -1;
    }

    y = std::atan2(static_cast<real_T>(u0_0), static_cast<real_T>(u1_0));
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = std::atan2(u0, u1);
  }

  return y;
}

void IMU_FusionModelClass::IMU_Fusion_ahrsfilter_stepImpl
  (fusion_simulink_ahrsfilter_IM_T *obj, const real_T accelIn[3], const real_T
   gyroIn[3], const real_T magIn[3], real_T orientOut[4], real_T av[3])
{
  static const int8_T tmp[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  static const int8_T tmp_0[9] = { -1, 0, 0, 0, -1, 0, 0, 0, -1 };

  real_T Ppost[144];
  real_T Qw[144];
  real_T H[72];
  real_T H_0[72];
  real_T K[72];
  real_T obj_0[72];
  real_T H_1[36];
  real_T xe_post[12];
  real_T Rprior[9];
  real_T h1[9];
  real_T h1_0[9];
  real_T ze[6];
  real_T gravityAccelGyroDiff[3];
  real_T offDiag[3];
  real_T absxk;
  real_T gyroOffsetErr_idx_0;
  real_T gyroOffsetErr_idx_1;
  real_T gyroOffsetErr_idx_2;
  real_T linAccelErr_idx_0;
  real_T linAccelErr_idx_1;
  real_T linAccelErr_idx_2;
  real_T s;
  real_T scale;
  real_T t;
  real_T x_a;
  real_T x_b;
  real_T x_c;
  real_T x_d;
  int32_T H_tmp;
  int32_T K_tmp;
  int32_T Ppost_tmp;
  int32_T aoffset;
  int32_T b_i;
  int32_T i;
  int32_T i_0;
  int32_T obj_tmp;
  boolean_T isJamming;
  IMUFusionCommon_computeAngularV(gyroIn, obj->pGyroOffset, av);
  if (obj->pFirstTime) {
    obj->pFirstTime = false;
    IMU_Fusion_NED_ecompass(accelIn, magIn, Rprior);
    IMU_Fusio_quaternion_quaternion(Rprior, &obj->pOrientPost.a,
      &obj->pOrientPost.b, &obj->pOrientPost.c, &obj->pOrientPost.d);
  }

  for (i_0 = 0; i_0 < 3; i_0++) {
    gravityAccelGyroDiff[i_0] = obj->pGyroOffset[i_0];
  }

  IMUFusionCommon_predictOrientat(obj, gyroIn, gravityAccelGyroDiff,
    obj->pOrientPost.a, obj->pOrientPost.b, obj->pOrientPost.c,
    obj->pOrientPost.d, &s, &scale, &absxk, &t);
  obj->pOrientPrior.d = t;
  obj->pOrientPrior.c = absxk;
  obj->pOrientPrior.b = scale;
  obj->pOrientPrior.a = s;
  IMU_Fusio_quaternionBase_rotmat(obj->pOrientPrior.a, obj->pOrientPrior.b,
    obj->pOrientPrior.c, obj->pOrientPrior.d, Rprior);
  for (b_i = 0; b_i < 3; b_i++) {
    obj->pLinAccelPrior[b_i] = obj->LinearAccelerationDecayFactor *
      obj->pLinAccelPost[b_i];
    s = Rprior[b_i + 6];
    gravityAccelGyroDiff[b_i] = (accelIn[b_i] + obj->pLinAccelPrior[b_i]) - s;
    offDiag[b_i] = 0.0;
    offDiag[b_i] += Rprior[b_i] * obj->pMagVec[0];
    offDiag[b_i] += Rprior[b_i + 3] * obj->pMagVec[1];
    offDiag[b_i] += s * obj->pMagVec[2];
  }

  std::memset(&h1[0], 0, 9U * sizeof(real_T));
  h1[3] = Rprior[8];
  h1[6] = -Rprior[7];
  h1[7] = Rprior[6];
  b_i = 0;
  for (aoffset = 0; aoffset < 3; aoffset++) {
    h1_0[b_i] = h1[b_i];
    h1_0[b_i + 1] = h1[b_i + 1] - h1[aoffset + 3];
    h1_0[b_i + 2] = h1[b_i + 2] - h1[aoffset + 6];
    b_i += 3;
  }

  for (b_i = 0; b_i < 9; b_i++) {
    h1[b_i] = h1_0[b_i];
    Rprior[b_i] = 0.0;
  }

  Rprior[3] = offDiag[2];
  Rprior[6] = -offDiag[1];
  Rprior[7] = offDiag[0];
  b_i = 0;
  for (aoffset = 0; aoffset < 3; aoffset++) {
    h1_0[b_i] = Rprior[b_i];
    h1_0[b_i + 1] = Rprior[b_i + 1] - Rprior[aoffset + 3];
    h1_0[b_i + 2] = Rprior[b_i + 2] - Rprior[aoffset + 6];
    b_i += 3;
  }

  std::memcpy(&Rprior[0], &h1_0[0], 9U * sizeof(real_T));
  b_i = 0;
  aoffset = 0;
  for (i_0 = 0; i_0 < 3; i_0++) {
    s = Rprior[b_i];
    scale = h1[b_i];
    H[aoffset] = scale;
    H[aoffset + 18] = -scale * obj->pKalmanPeriod;
    H[aoffset + 36] = tmp[b_i];
    H[aoffset + 54] = 0.0;
    H[aoffset + 3] = s;
    H[aoffset + 21] = -s * obj->pKalmanPeriod;
    H[aoffset + 39] = 0.0;
    H[aoffset + 57] = tmp_0[b_i];
    s = Rprior[b_i + 1];
    scale = h1[b_i + 1];
    H[aoffset + 1] = scale;
    H[aoffset + 19] = -scale * obj->pKalmanPeriod;
    H[aoffset + 37] = tmp[b_i + 1];
    H[aoffset + 55] = 0.0;
    H[aoffset + 4] = s;
    H[aoffset + 22] = -s * obj->pKalmanPeriod;
    H[aoffset + 40] = 0.0;
    H[aoffset + 58] = tmp_0[b_i + 1];
    s = Rprior[b_i + 2];
    scale = h1[b_i + 2];
    H[aoffset + 2] = scale;
    H[aoffset + 20] = -scale * obj->pKalmanPeriod;
    H[aoffset + 38] = tmp[b_i + 2];
    H[aoffset + 56] = 0.0;
    H[aoffset + 5] = s;
    H[aoffset + 23] = -s * obj->pKalmanPeriod;
    H[aoffset + 41] = 0.0;
    H[aoffset + 59] = tmp_0[b_i + 2];
    b_i += 3;
    aoffset += 6;
  }

  std::memcpy(&Qw[0], &obj->pQw[0], 144U * sizeof(real_T));
  b_i = 0;
  for (aoffset = 0; aoffset < 6; aoffset++) {
    i_0 = 0;
    obj_tmp = 0;
    for (Ppost_tmp = 0; Ppost_tmp < 12; Ppost_tmp++) {
      K_tmp = i_0 + aoffset;
      K[Ppost_tmp + b_i] = H[K_tmp];
      H_0[K_tmp] = 0.0;
      i = 0;
      for (H_tmp = 0; H_tmp < 12; H_tmp++) {
        H_0[K_tmp] += H[i + aoffset] * obj->pQw[H_tmp + obj_tmp];
        i += 6;
      }

      i_0 += 6;
      obj_tmp += 12;
    }

    b_i += 12;
  }

  for (b_i = 0; b_i < 6; b_i++) {
    for (aoffset = 0; aoffset < 12; aoffset++) {
      obj_tmp = aoffset + 12 * b_i;
      obj_0[obj_tmp] = 0.0;
      for (i_0 = 0; i_0 < 12; i_0++) {
        obj_0[obj_tmp] += obj->pQw[12 * i_0 + aoffset] * K[12 * b_i + i_0];
      }
    }

    for (aoffset = 0; aoffset < 6; aoffset++) {
      s = 0.0;
      for (i_0 = 0; i_0 < 12; i_0++) {
        s += H_0[6 * i_0 + aoffset] * K[12 * b_i + i_0];
      }

      H_1[b_i + 6 * aoffset] = obj->pQv[6 * b_i + aoffset] + s;
    }
  }

  IMU_Fusion_mrdiv(obj_0, H_1, K);
  ze[0] = gravityAccelGyroDiff[0];
  ze[3] = magIn[0] - offDiag[0];
  ze[1] = gravityAccelGyroDiff[1];
  ze[4] = magIn[1] - offDiag[1];
  ze[2] = gravityAccelGyroDiff[2];
  ze[5] = magIn[2] - offDiag[2];
  for (b_i = 0; b_i < 3; b_i++) {
    offDiag[b_i] = 0.0;
    aoffset = 0;
    for (i_0 = 0; i_0 < 6; i_0++) {
      offDiag[b_i] += K[(aoffset + b_i) + 9] * ze[i_0];
      aoffset += 12;
    }
  }

  scale = 3.3121686421112381E-170;
  absxk = std::abs(offDiag[0]);
  if (absxk > 3.3121686421112381E-170) {
    s = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    s = t * t;
  }

  absxk = std::abs(offDiag[1]);
  if (absxk > scale) {
    t = scale / absxk;
    s = s * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    s += t * t;
  }

  absxk = std::abs(offDiag[2]);
  if (absxk > scale) {
    t = scale / absxk;
    s = s * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    s += t * t;
  }

  s = scale * std::sqrt(s);
  isJamming = (s * s > obj->ExpectedMagneticFieldStrength *
               obj->ExpectedMagneticFieldStrength * 4.0);
  if (isJamming) {
    for (b_i = 0; b_i < 9; b_i++) {
      h1[b_i] = 0.0;
      h1[b_i] += K[b_i] * gravityAccelGyroDiff[0];
      h1[b_i] += K[b_i + 12] * gravityAccelGyroDiff[1];
      h1[b_i] += K[b_i + 24] * gravityAccelGyroDiff[2];
    }

    gravityAccelGyroDiff[0] = h1[0];
    gyroOffsetErr_idx_0 = h1[3];
    linAccelErr_idx_0 = h1[6];
    gravityAccelGyroDiff[1] = h1[1];
    gyroOffsetErr_idx_1 = h1[4];
    linAccelErr_idx_1 = h1[7];
    gravityAccelGyroDiff[2] = h1[2];
    gyroOffsetErr_idx_2 = h1[5];
    linAccelErr_idx_2 = h1[8];
  } else {
    for (b_i = 0; b_i < 12; b_i++) {
      xe_post[b_i] = 0.0;
      aoffset = 0;
      for (i_0 = 0; i_0 < 6; i_0++) {
        xe_post[b_i] += K[aoffset + b_i] * ze[i_0];
        aoffset += 12;
      }
    }

    gravityAccelGyroDiff[0] = xe_post[0];
    gyroOffsetErr_idx_0 = xe_post[3];
    linAccelErr_idx_0 = xe_post[6];
    gravityAccelGyroDiff[1] = xe_post[1];
    gyroOffsetErr_idx_1 = xe_post[4];
    linAccelErr_idx_1 = xe_post[7];
    gravityAccelGyroDiff[2] = xe_post[2];
    gyroOffsetErr_idx_2 = xe_post[5];
    linAccelErr_idx_2 = xe_post[8];
  }

  IMU_Fus_quaternion_quaternion_o(gravityAccelGyroDiff, &s, &scale, &absxk, &t);
  scale = -scale;
  absxk = -absxk;
  t = -t;
  x_a = obj->pOrientPrior.a;
  x_b = obj->pOrientPrior.b;
  x_c = obj->pOrientPrior.c;
  x_d = obj->pOrientPrior.d;
  obj->pOrientPost.a = ((obj->pOrientPrior.a * s - obj->pOrientPrior.b * scale)
                        - obj->pOrientPrior.c * absxk) - obj->pOrientPrior.d * t;
  obj->pOrientPost.b = ((x_a * scale + x_b * s) + x_c * t) - x_d * absxk;
  obj->pOrientPost.c = ((x_a * absxk - x_b * t) + x_c * s) + x_d * scale;
  obj->pOrientPost.d = ((x_a * t + x_b * absxk) - x_c * scale) + x_d * s;
  if (obj->pOrientPost.a < 0.0) {
    s = obj->pOrientPost.b;
    scale = obj->pOrientPost.c;
    absxk = obj->pOrientPost.d;
    obj->pOrientPost.a = -obj->pOrientPost.a;
    obj->pOrientPost.b = -s;
    obj->pOrientPost.c = -scale;
    obj->pOrientPost.d = -absxk;
  }

  s = obj->pOrientPost.b;
  scale = obj->pOrientPost.c;
  absxk = obj->pOrientPost.d;
  t = std::sqrt(((obj->pOrientPost.a * obj->pOrientPost.a + obj->pOrientPost.b *
                  obj->pOrientPost.b) + obj->pOrientPost.c * obj->pOrientPost.c)
                + obj->pOrientPost.d * obj->pOrientPost.d);
  obj->pOrientPost.a /= t;
  obj->pOrientPost.b = s / t;
  obj->pOrientPost.c = scale / t;
  obj->pOrientPost.d = absxk / t;
  IMU_Fusio_quaternionBase_rotmat(obj->pOrientPost.a, obj->pOrientPost.b,
    obj->pOrientPost.c, obj->pOrientPost.d, h1);
  obj->pGyroOffset[0] -= gyroOffsetErr_idx_0;
  obj->pLinAccelPost[0] = obj->pLinAccelPrior[0] - linAccelErr_idx_0;
  obj->pGyroOffset[1] -= gyroOffsetErr_idx_1;
  obj->pLinAccelPost[1] = obj->pLinAccelPrior[1] - linAccelErr_idx_1;
  obj->pGyroOffset[2] -= gyroOffsetErr_idx_2;
  obj->pLinAccelPost[2] = obj->pLinAccelPrior[2] - linAccelErr_idx_2;
  if (!isJamming) {
    for (b_i = 0; b_i < 3; b_i++) {
      aoffset = b_i * 3 - 1;
      gravityAccelGyroDiff[b_i] = (h1[aoffset + 1] * offDiag[0] + h1[aoffset + 2]
        * offDiag[1]) + h1[aoffset + 3] * offDiag[2];
    }

    gravityAccelGyroDiff[0] = obj->pMagVec[0] - gravityAccelGyroDiff[0];
    s = rt_atan2d_snf(obj->pMagVec[2] - gravityAccelGyroDiff[2],
                      gravityAccelGyroDiff[0]);
    if (s < -1.5707963267948966) {
      s = -1.5707963267948966;
    }

    if (s > 1.5707963267948966) {
      s = 1.5707963267948966;
    }

    obj->pMagVec[0] = 0.0;
    obj->pMagVec[1] = 0.0;
    obj->pMagVec[2] = 0.0;
    obj->pMagVec[0] = std::cos(s);
    obj->pMagVec[2] = std::sin(s);
    obj->pMagVec[0] *= obj->ExpectedMagneticFieldStrength;
    obj->pMagVec[1] *= obj->ExpectedMagneticFieldStrength;
    obj->pMagVec[2] *= obj->ExpectedMagneticFieldStrength;
  }

  b_i = 0;
  aoffset = 0;
  for (i_0 = 0; i_0 < 12; i_0++) {
    for (obj_tmp = 0; obj_tmp < 6; obj_tmp++) {
      H_tmp = obj_tmp + b_i;
      H_0[H_tmp] = 0.0;
      Ppost_tmp = 0;
      for (i = 0; i < 12; i++) {
        H_0[H_tmp] += H[Ppost_tmp + obj_tmp] * Qw[i + aoffset];
        Ppost_tmp += 6;
      }
    }

    b_i += 6;
    aoffset += 12;
  }

  for (b_i = 0; b_i < 12; b_i++) {
    aoffset = 0;
    i_0 = 0;
    for (obj_tmp = 0; obj_tmp < 12; obj_tmp++) {
      s = 0.0;
      Ppost_tmp = 0;
      for (i = 0; i < 6; i++) {
        s += K[Ppost_tmp + b_i] * H_0[i + i_0];
        Ppost_tmp += 12;
      }

      Ppost_tmp = aoffset + b_i;
      Ppost[Ppost_tmp] = Qw[Ppost_tmp] - s;
      aoffset += 12;
      i_0 += 6;
    }
  }

  std::memset(&Qw[0], 0, 144U * sizeof(real_T));
  s = obj->pKalmanPeriod * obj->pKalmanPeriod;
  scale = obj->GyroscopeDriftNoise + obj->GyroscopeNoise;
  Qw[0] = (Ppost[39] + scale) * s + Ppost[0];
  Qw[39] = Ppost[39] + obj->GyroscopeDriftNoise;
  offDiag[0] = -obj->pKalmanPeriod * Qw[39];
  Qw[13] = (Ppost[52] + scale) * s + Ppost[13];
  Qw[52] = Ppost[52] + obj->GyroscopeDriftNoise;
  offDiag[1] = -obj->pKalmanPeriod * Qw[52];
  Qw[26] = (Ppost[65] + scale) * s + Ppost[26];
  Qw[65] = Ppost[65] + obj->GyroscopeDriftNoise;
  offDiag[2] = -obj->pKalmanPeriod * Qw[65];
  Qw[3] = offDiag[0];
  Qw[16] = offDiag[1];
  Qw[29] = offDiag[2];
  Qw[36] = offDiag[0];
  Qw[49] = offDiag[1];
  Qw[62] = offDiag[2];
  s = obj->LinearAccelerationDecayFactor * obj->LinearAccelerationDecayFactor;
  Qw[78] = s * Ppost[78] + obj->LinearAccelerationNoise;
  Qw[91] = s * Ppost[91] + obj->LinearAccelerationNoise;
  Qw[104] = s * Ppost[104] + obj->LinearAccelerationNoise;
  s = obj->MagneticDisturbanceDecayFactor * obj->MagneticDisturbanceDecayFactor;
  Qw[117] = s * Ppost[117] + obj->MagneticDisturbanceNoise;
  Qw[130] = s * Ppost[130] + obj->MagneticDisturbanceNoise;
  Qw[143] = s * Ppost[143] + obj->MagneticDisturbanceNoise;
  std::memcpy(&obj->pQw[0], &Qw[0], 144U * sizeof(real_T));
  orientOut[0] = obj->pOrientPost.a;
  orientOut[1] = obj->pOrientPost.b;
  orientOut[2] = obj->pOrientPost.c;
  orientOut[3] = obj->pOrientPost.d;
}

void IMU_FusionModelClass::IMU_Fu_AHRSFilterBase_resetImpl
  (fusion_simulink_ahrsfilter_IM_T *obj)
{
  static const real_T tmp[144] = { 6.0923483957341713E-6, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0923483957341713E-6, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0923483957341713E-6, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 7.6154354946677142E-5, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 7.6154354946677142E-5,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    7.6154354946677142E-5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0096236100000000012, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0096236100000000012, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0096236100000000012, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.6 };

  static const int8_T tmp_3[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  real_T accelMeasNoiseVar;
  real_T magMeasNoiseVar;
  int32_T i;
  int32_T tmp_0;
  int32_T tmp_1;
  int32_T tmp_2;
  obj->pOrientPost.a = 1.0;
  obj->pOrientPost.b = 0.0;
  obj->pOrientPost.c = 0.0;
  obj->pOrientPost.d = 0.0;
  obj->pGyroOffset[0] = 0.0;
  obj->pMagVec[0] = 0.0;
  obj->pGyroOffset[1] = 0.0;
  obj->pMagVec[1] = 0.0;
  obj->pGyroOffset[2] = 0.0;
  obj->pMagVec[2] = 0.0;
  obj->pMagVec[0] = obj->ExpectedMagneticFieldStrength;
  magMeasNoiseVar = obj->pKalmanPeriod * obj->pKalmanPeriod *
    (obj->GyroscopeDriftNoise + obj->GyroscopeNoise);
  accelMeasNoiseVar = magMeasNoiseVar + (obj->AccelerometerNoise +
    obj->LinearAccelerationNoise);
  magMeasNoiseVar += obj->MagnetometerNoise + obj->MagneticDisturbanceNoise;
  std::memset(&obj->pQv[0], 0, 36U * sizeof(real_T));
  tmp_1 = 0;
  tmp_2 = 0;
  for (i = 0; i < 3; i++) {
    tmp_0 = tmp_3[tmp_1];
    obj->pQv[tmp_2] = accelMeasNoiseVar * static_cast<real_T>(tmp_0);
    obj->pQv[tmp_2 + 21] = magMeasNoiseVar * static_cast<real_T>(tmp_0);
    tmp_0 = tmp_3[tmp_1 + 1];
    obj->pQv[tmp_2 + 1] = accelMeasNoiseVar * static_cast<real_T>(tmp_0);
    obj->pQv[tmp_2 + 22] = magMeasNoiseVar * static_cast<real_T>(tmp_0);
    tmp_0 = tmp_3[tmp_1 + 2];
    obj->pQv[tmp_2 + 2] = accelMeasNoiseVar * static_cast<real_T>(tmp_0);
    obj->pQv[tmp_2 + 23] = magMeasNoiseVar * static_cast<real_T>(tmp_0);
    tmp_1 += 3;
    tmp_2 += 6;
  }

  std::memcpy(&obj->pQw[0], &tmp[0], 144U * sizeof(real_T));
  obj->pLinAccelPost[0] = 0.0;
  obj->pLinAccelPost[1] = 0.0;
  obj->pLinAccelPost[2] = 0.0;
  obj->pFirstTime = true;
}

// Model step function
void IMU_FusionModelClass::step()
{
  static const int8_T tmp_4[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  real_T y[36];
  real_T b_varargout_1[4];
  real_T b_varargout_2[3];
  real_T tmp[3];
  real_T tmp_0[3];
  real_T Euler_tmp;
  real_T Euler_tmp_0;
  real_T Euler_tmp_1;
  real_T Euler_tmp_2;
  real_T accelMeasNoiseVar;
  real_T magMeasNoiseVar;
  real_T rtb_Product1;
  real_T rtb_Product2;
  real_T rtb_fcn3;
  int32_T i;
  int32_T tmp_1;
  int32_T tmp_2;
  int32_T tmp_3;
  boolean_T flag;

  // MATLABSystem: '<Root>/AHRS'
  // Unit Conversion - from: deg/s to: rad/s
  // Expression: output = (0.0174533*input) + (0)
  if (IMU_Fusion_DW.obj.AccelerometerNoise !=
      IMU_Fusion_P.AHRS_AccelerometerNoise) {
    flag = (IMU_Fusion_DW.obj.isInitialized == 1);
    if (flag) {
      IMU_Fusion_DW.obj.TunablePropsChanged = true;
    }

    IMU_Fusion_DW.obj.AccelerometerNoise = IMU_Fusion_P.AHRS_AccelerometerNoise;
  }

  if (IMU_Fusion_DW.obj.GyroscopeNoise != IMU_Fusion_P.AHRS_GyroscopeNoise) {
    flag = (IMU_Fusion_DW.obj.isInitialized == 1);
    if (flag) {
      IMU_Fusion_DW.obj.TunablePropsChanged = true;
    }

    IMU_Fusion_DW.obj.GyroscopeNoise = IMU_Fusion_P.AHRS_GyroscopeNoise;
  }

  if (IMU_Fusion_DW.obj.MagnetometerNoise != IMU_Fusion_P.AHRS_MagnetometerNoise)
  {
    flag = (IMU_Fusion_DW.obj.isInitialized == 1);
    if (flag) {
      IMU_Fusion_DW.obj.TunablePropsChanged = true;
    }

    IMU_Fusion_DW.obj.MagnetometerNoise = IMU_Fusion_P.AHRS_MagnetometerNoise;
  }

  if (IMU_Fusion_DW.obj.GyroscopeDriftNoise !=
      IMU_Fusion_P.AHRS_GyroscopeDriftNoise) {
    flag = (IMU_Fusion_DW.obj.isInitialized == 1);
    if (flag) {
      IMU_Fusion_DW.obj.TunablePropsChanged = true;
    }

    IMU_Fusion_DW.obj.GyroscopeDriftNoise =
      IMU_Fusion_P.AHRS_GyroscopeDriftNoise;
  }

  if (IMU_Fusion_DW.obj.LinearAccelerationNoise !=
      IMU_Fusion_P.AHRS_LinearAccelerationNoise) {
    flag = (IMU_Fusion_DW.obj.isInitialized == 1);
    if (flag) {
      IMU_Fusion_DW.obj.TunablePropsChanged = true;
    }

    IMU_Fusion_DW.obj.LinearAccelerationNoise =
      IMU_Fusion_P.AHRS_LinearAccelerationNoise;
  }

  if (IMU_Fusion_DW.obj.MagneticDisturbanceNoise !=
      IMU_Fusion_P.AHRS_MagneticDisturbanceNoise) {
    flag = (IMU_Fusion_DW.obj.isInitialized == 1);
    if (flag) {
      IMU_Fusion_DW.obj.TunablePropsChanged = true;
    }

    IMU_Fusion_DW.obj.MagneticDisturbanceNoise =
      IMU_Fusion_P.AHRS_MagneticDisturbanceNoise;
  }

  if (IMU_Fusion_DW.obj.LinearAccelerationDecayFactor !=
      IMU_Fusion_P.AHRS_LinearAccelerationDecayFac) {
    flag = (IMU_Fusion_DW.obj.isInitialized == 1);
    if (flag) {
      IMU_Fusion_DW.obj.TunablePropsChanged = true;
    }

    IMU_Fusion_DW.obj.LinearAccelerationDecayFactor =
      IMU_Fusion_P.AHRS_LinearAccelerationDecayFac;
  }

  if (IMU_Fusion_DW.obj.MagneticDisturbanceDecayFactor !=
      IMU_Fusion_P.AHRS_MagneticDisturbanceDecayFa) {
    flag = (IMU_Fusion_DW.obj.isInitialized == 1);
    if (flag) {
      IMU_Fusion_DW.obj.TunablePropsChanged = true;
    }

    IMU_Fusion_DW.obj.MagneticDisturbanceDecayFactor =
      IMU_Fusion_P.AHRS_MagneticDisturbanceDecayFa;
  }

  if (IMU_Fusion_DW.obj.ExpectedMagneticFieldStrength !=
      IMU_Fusion_P.AHRS_ExpectedMagneticFieldStren) {
    flag = (IMU_Fusion_DW.obj.isInitialized == 1);
    if (flag) {
      IMU_Fusion_DW.obj.TunablePropsChanged = true;
    }

    IMU_Fusion_DW.obj.ExpectedMagneticFieldStrength =
      IMU_Fusion_P.AHRS_ExpectedMagneticFieldStren;
  }

  if (IMU_Fusion_DW.obj.TunablePropsChanged) {
    IMU_Fusion_DW.obj.TunablePropsChanged = false;
    magMeasNoiseVar = IMU_Fusion_DW.obj.pKalmanPeriod;
    rtb_Product1 = IMU_Fusion_DW.obj.GyroscopeDriftNoise +
      IMU_Fusion_DW.obj.GyroscopeNoise;
    accelMeasNoiseVar = magMeasNoiseVar * magMeasNoiseVar * rtb_Product1 +
      (IMU_Fusion_DW.obj.AccelerometerNoise +
       IMU_Fusion_DW.obj.LinearAccelerationNoise);
    magMeasNoiseVar = IMU_Fusion_DW.obj.pKalmanPeriod;
    magMeasNoiseVar = magMeasNoiseVar * magMeasNoiseVar * rtb_Product1 +
      (IMU_Fusion_DW.obj.MagnetometerNoise +
       IMU_Fusion_DW.obj.MagneticDisturbanceNoise);
    std::memset(&y[0], 0, 36U * sizeof(real_T));
    tmp_2 = 0;
    tmp_3 = 0;
    for (i = 0; i < 3; i++) {
      tmp_1 = tmp_4[tmp_2];
      y[tmp_3] = accelMeasNoiseVar * static_cast<real_T>(tmp_1);
      y[tmp_3 + 21] = magMeasNoiseVar * static_cast<real_T>(tmp_1);
      tmp_1 = tmp_4[tmp_2 + 1];
      y[tmp_3 + 1] = accelMeasNoiseVar * static_cast<real_T>(tmp_1);
      y[tmp_3 + 22] = magMeasNoiseVar * static_cast<real_T>(tmp_1);
      tmp_1 = tmp_4[tmp_2 + 2];
      y[tmp_3 + 2] = accelMeasNoiseVar * static_cast<real_T>(tmp_1);
      y[tmp_3 + 23] = magMeasNoiseVar * static_cast<real_T>(tmp_1);
      tmp_2 += 3;
      tmp_3 += 6;
    }

    std::memcpy(&IMU_Fusion_DW.obj.pQv[0], &y[0], 36U * sizeof(real_T));
  }

  // UnitConversion: '<S1>/Unit Conversion' incorporates:
  //   Inport: '<Root>/Gyro_Raw'

  tmp[0] = 0.017453292519943295 * IMU_Fusion_U.Gyro_Raw[0];

  // Product: '<Root>/Divide' incorporates:
  //   Constant: '<Root>/Constant'
  //   Inport: '<Root>/Mag_Raw'

  tmp_0[0] = IMU_Fusion_U.Mag_Raw[0] / IMU_Fusion_P.Constant_Value_e;

  // UnitConversion: '<S1>/Unit Conversion' incorporates:
  //   Inport: '<Root>/Gyro_Raw'

  tmp[1] = 0.017453292519943295 * IMU_Fusion_U.Gyro_Raw[1];

  // Product: '<Root>/Divide' incorporates:
  //   Constant: '<Root>/Constant'
  //   Inport: '<Root>/Mag_Raw'

  tmp_0[1] = IMU_Fusion_U.Mag_Raw[1] / IMU_Fusion_P.Constant_Value_e;

  // UnitConversion: '<S1>/Unit Conversion' incorporates:
  //   Inport: '<Root>/Gyro_Raw'

  tmp[2] = 0.017453292519943295 * IMU_Fusion_U.Gyro_Raw[2];

  // Product: '<Root>/Divide' incorporates:
  //   Constant: '<Root>/Constant'
  //   Inport: '<Root>/Mag_Raw'

  tmp_0[2] = IMU_Fusion_U.Mag_Raw[2] / IMU_Fusion_P.Constant_Value_e;

  // MATLABSystem: '<Root>/AHRS' incorporates:
  //   Inport: '<Root>/Accel_Raw'

  IMU_Fusion_ahrsfilter_stepImpl(&IMU_Fusion_DW.obj, IMU_Fusion_U.Accel_Raw, tmp,
    tmp_0, b_varargout_1, b_varargout_2);

  // Outport: '<Root>/Quat' incorporates:
  //   MATLABSystem: '<Root>/AHRS'

  IMU_Fusion_Y.Quat[0] = b_varargout_1[0];
  IMU_Fusion_Y.Quat[1] = b_varargout_1[1];
  IMU_Fusion_Y.Quat[2] = b_varargout_1[2];
  IMU_Fusion_Y.Quat[3] = b_varargout_1[3];

  // Outport: '<Root>/AngVel' incorporates:
  //   MATLABSystem: '<Root>/AHRS'

  IMU_Fusion_Y.AngVel[0] = b_varargout_2[0];
  IMU_Fusion_Y.AngVel[1] = b_varargout_2[1];
  IMU_Fusion_Y.AngVel[2] = b_varargout_2[2];

  // Sqrt: '<S9>/sqrt' incorporates:
  //   MATLABSystem: '<Root>/AHRS'
  //   Product: '<S10>/Product'
  //   Product: '<S10>/Product1'
  //   Product: '<S10>/Product2'
  //   Product: '<S10>/Product3'
  //   Sum: '<S10>/Sum'

  accelMeasNoiseVar = std::sqrt(((b_varargout_1[0] * b_varargout_1[0] +
    b_varargout_1[1] * b_varargout_1[1]) + b_varargout_1[2] * b_varargout_1[2])
    + b_varargout_1[3] * b_varargout_1[3]);

  // Product: '<S4>/Product' incorporates:
  //   MATLABSystem: '<Root>/AHRS'

  magMeasNoiseVar = b_varargout_1[0] / accelMeasNoiseVar;

  // Product: '<S4>/Product1' incorporates:
  //   MATLABSystem: '<Root>/AHRS'

  rtb_Product1 = b_varargout_1[1] / accelMeasNoiseVar;

  // Product: '<S4>/Product2' incorporates:
  //   MATLABSystem: '<Root>/AHRS'

  rtb_Product2 = b_varargout_1[2] / accelMeasNoiseVar;

  // Product: '<S4>/Product3' incorporates:
  //   MATLABSystem: '<Root>/AHRS'

  accelMeasNoiseVar = b_varargout_1[3] / accelMeasNoiseVar;

  // Fcn: '<S2>/fcn2' incorporates:
  //   Fcn: '<S2>/fcn5'

  Euler_tmp = magMeasNoiseVar * magMeasNoiseVar;
  Euler_tmp_0 = rtb_Product1 * rtb_Product1;
  Euler_tmp_1 = rtb_Product2 * rtb_Product2;
  Euler_tmp_2 = accelMeasNoiseVar * accelMeasNoiseVar;

  // Trigonometry: '<S3>/Trigonometric Function1' incorporates:
  //   Fcn: '<S2>/fcn1'
  //   Fcn: '<S2>/fcn2'
  //   Outport: '<Root>/Euler'

  IMU_Fusion_Y.Euler[0] = rt_atan2d_snf((rtb_Product1 * rtb_Product2 +
    magMeasNoiseVar * accelMeasNoiseVar) * 2.0, ((Euler_tmp + Euler_tmp_0) -
    Euler_tmp_1) - Euler_tmp_2);

  // Fcn: '<S2>/fcn3'
  rtb_fcn3 = (rtb_Product1 * accelMeasNoiseVar - magMeasNoiseVar * rtb_Product2)
    * -2.0;

  // Trigonometry: '<S3>/Trigonometric Function3' incorporates:
  //   Fcn: '<S2>/fcn4'
  //   Fcn: '<S2>/fcn5'
  //   Outport: '<Root>/Euler'

  IMU_Fusion_Y.Euler[2] = rt_atan2d_snf((rtb_Product2 * accelMeasNoiseVar +
    magMeasNoiseVar * rtb_Product1) * 2.0, ((Euler_tmp - Euler_tmp_0) -
    Euler_tmp_1) + Euler_tmp_2);

  // If: '<S5>/If' incorporates:
  //   Constant: '<S6>/Constant'
  //   Constant: '<S7>/Constant'

  if (rtb_fcn3 > 1.0) {
    rtb_fcn3 = IMU_Fusion_P.Constant_Value;
  } else {
    if (rtb_fcn3 < -1.0) {
      rtb_fcn3 = IMU_Fusion_P.Constant_Value_l;
    }
  }

  // End of If: '<S5>/If'

  // Trigonometry: '<S3>/trigFcn' incorporates:
  //   Outport: '<Root>/Euler'

  if (rtb_fcn3 > 1.0) {
    rtb_fcn3 = 1.0;
  } else {
    if (rtb_fcn3 < -1.0) {
      rtb_fcn3 = -1.0;
    }
  }

  IMU_Fusion_Y.Euler[1] = std::asin(rtb_fcn3);

  // End of Trigonometry: '<S3>/trigFcn'
}

// Model initialize function
void IMU_FusionModelClass::initialize()
{
  // Registration code

  // initialize non-finites
  rt_InitInfAndNaN(sizeof(real_T));

  {
    fusion_simulink_ahrsfilter_IM_T *obj;
    boolean_T flag;

    // Start for MATLABSystem: '<Root>/AHRS' incorporates:
    //   Inport: '<Root>/Accel_Raw'

    IMU_Fusion_DW.obj.isInitialized = 0;
    flag = (IMU_Fusion_DW.obj.isInitialized == 1);
    if (flag) {
      IMU_Fusion_DW.obj.TunablePropsChanged = true;
    }

    IMU_Fusion_DW.obj.AccelerometerNoise = IMU_Fusion_P.AHRS_AccelerometerNoise;
    flag = (IMU_Fusion_DW.obj.isInitialized == 1);
    if (flag) {
      IMU_Fusion_DW.obj.TunablePropsChanged = true;
    }

    IMU_Fusion_DW.obj.GyroscopeNoise = IMU_Fusion_P.AHRS_GyroscopeNoise;
    flag = (IMU_Fusion_DW.obj.isInitialized == 1);
    if (flag) {
      IMU_Fusion_DW.obj.TunablePropsChanged = true;
    }

    IMU_Fusion_DW.obj.MagnetometerNoise = IMU_Fusion_P.AHRS_MagnetometerNoise;
    flag = (IMU_Fusion_DW.obj.isInitialized == 1);
    if (flag) {
      IMU_Fusion_DW.obj.TunablePropsChanged = true;
    }

    IMU_Fusion_DW.obj.GyroscopeDriftNoise =
      IMU_Fusion_P.AHRS_GyroscopeDriftNoise;
    flag = (IMU_Fusion_DW.obj.isInitialized == 1);
    if (flag) {
      IMU_Fusion_DW.obj.TunablePropsChanged = true;
    }

    IMU_Fusion_DW.obj.LinearAccelerationNoise =
      IMU_Fusion_P.AHRS_LinearAccelerationNoise;
    flag = (IMU_Fusion_DW.obj.isInitialized == 1);
    if (flag) {
      IMU_Fusion_DW.obj.TunablePropsChanged = true;
    }

    IMU_Fusion_DW.obj.MagneticDisturbanceNoise =
      IMU_Fusion_P.AHRS_MagneticDisturbanceNoise;
    flag = (IMU_Fusion_DW.obj.isInitialized == 1);
    if (flag) {
      IMU_Fusion_DW.obj.TunablePropsChanged = true;
    }

    IMU_Fusion_DW.obj.LinearAccelerationDecayFactor =
      IMU_Fusion_P.AHRS_LinearAccelerationDecayFac;
    flag = (IMU_Fusion_DW.obj.isInitialized == 1);
    if (flag) {
      IMU_Fusion_DW.obj.TunablePropsChanged = true;
    }

    IMU_Fusion_DW.obj.MagneticDisturbanceDecayFactor =
      IMU_Fusion_P.AHRS_MagneticDisturbanceDecayFa;
    flag = (IMU_Fusion_DW.obj.isInitialized == 1);
    if (flag) {
      IMU_Fusion_DW.obj.TunablePropsChanged = true;
    }

    IMU_Fusion_DW.obj.ExpectedMagneticFieldStrength =
      IMU_Fusion_P.AHRS_ExpectedMagneticFieldStren;
    obj = &IMU_Fusion_DW.obj;
    IMU_Fusion_DW.obj.isInitialized = 1;
    IMU_Fusion_DW.obj.pInputPrototype[0] = IMU_Fusion_U.Accel_Raw[0];
    IMU_Fusion_DW.obj.pInputPrototype[1] = IMU_Fusion_U.Accel_Raw[1];
    IMU_Fusion_DW.obj.pInputPrototype[2] = IMU_Fusion_U.Accel_Raw[2];
    IMU_Fusion_DW.obj.pSensorPeriod = 0.2;
    IMU_Fusion_DW.obj.pKalmanPeriod = IMU_Fusion_DW.obj.pSensorPeriod;
    IMU_Fusion_DW.obj.pRefSys = &obj->_pobj0;
    IMU_Fusion_DW.obj.TunablePropsChanged = false;

    // End of Start for MATLABSystem: '<Root>/AHRS'

    // InitializeConditions for MATLABSystem: '<Root>/AHRS'
    IMU_Fu_AHRSFilterBase_resetImpl(&IMU_Fusion_DW.obj);
  }
}

// Model terminate function
void IMU_FusionModelClass::terminate()
{
  // (no terminate code required)
}

// Constructor
IMU_FusionModelClass::IMU_FusionModelClass() :
  IMU_Fusion_DW(),
  IMU_Fusion_U(),
  IMU_Fusion_Y(),
  IMU_Fusion_M()
{
  // Currently there is no constructor body generated.
}

// Destructor
IMU_FusionModelClass::~IMU_FusionModelClass()
{
  // Currently there is no destructor body generated.
}

// Real-Time Model get method
IMU_FusionModelClass::RT_MODEL_IMU_Fusion_T * IMU_FusionModelClass::getRTM()
{
  return (&IMU_Fusion_M);
}

//
// File trailer for generated code.
//
// [EOF]
//
