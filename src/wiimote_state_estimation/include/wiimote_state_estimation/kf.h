#ifndef WIIMOTE_STATE_ESTIMATION_KF_H
#define WIIMOTE_STATE_ESTIMATION_KF_H

#include <Eigen/Dense>
#include "wiimote_state_estimation/filter_method.h"

/**
 * Template class that uses the linear Kalman Filter algorithm to filter noisy measurement data.
 * @param NX Number of states
 * @param NU Number of inputs
 * @param NZ Number of measurements
 */
template <unsigned int NX, unsigned int NU, unsigned int NZ>
class KalmanFilter : public FilterMethod<NX, NU, NZ>
{
public:
  /**
 * Initialises the filter with the relevant matrices.
 * @param x_pri_init Inital state
 * @param S_init Initial covariance matrix
 * @param A The A matrix in the linear state space model x_k+1 = Ax_k + Bu_k
 * @param B The B matrix in the linear state space model x_k+1 = Ax_k + Bu_k
 * @param C The C matrix in the linear observation model z_k = Cx_k
 * @param R The covariance matrix of the process noise
 * @param Q The covariance matrix of the measurement noise
 */
  KalmanFilter(const Eigen::Matrix<double, NX, 1> &x_pri_init,
               const Eigen::Matrix<double, NX, NX> &S_init,
               const Eigen::Matrix<double, NX, NX> &A,
               const Eigen::Matrix<double, NX, NU> &B,
               const Eigen::Matrix<double, NZ, NX> &C,
               const Eigen::Matrix<double, NX, NX> &R,
               const Eigen::Matrix<double, NZ, NZ> &Q);

  KalmanFilter() = default;

  /**
   * Run one iteration of the Kalman Filter algorithm, given a current input and measurement.
   * @param u Current input.
   * @param z Current measurement.
   */
  virtual void iterate(const Eigen::Matrix<double, NU, 1> &u, const Eigen::Matrix<double, NZ, 1> &z) override;

  /**
   * Get current a posteriori state.
   * @return const Eigen::Matrix<double, NX, 1>& Constant reference to the internal x_post.
   */
  virtual const Eigen::Matrix<double, NX, 1> &getState() const override { return x_post; }

  /**
   * Get current a posteriori covariance matrix.
   * @return const Eigen::Matrix<double, NX, 1>& Constant reference to the internal S_post.
   */
  virtual const Eigen::Matrix<double, NX, NX> &getCoVar() const override { return S_post; }

private:
  Eigen::Matrix<double, NX, 1> x_pri;
  Eigen::Matrix<double, NX, 1> x_post;
  Eigen::Matrix<double, NX, NX> S_pri;
  Eigen::Matrix<double, NX, NX> S_post;
  Eigen::Matrix<double, NX, NX> A;
  Eigen::Matrix<double, NX, NU> B;
  Eigen::Matrix<double, NZ, NX> C;
  Eigen::Matrix<double, NX, NZ> K;
  Eigen::Matrix<double, NX, NX> R;
  Eigen::Matrix<double, NZ, NZ> Q;

  void update_x_pri(const Eigen::Matrix<double, NU, 1> &u);
  void update_x_post(const Eigen::Matrix<double, NZ, 1> &z);
  void update_S_pri();
  void update_S_post();
  void update_K();
};

template <unsigned int NX, unsigned int NU, unsigned int NZ>
KalmanFilter<NX, NU, NZ>::KalmanFilter(const Eigen::Matrix<double, NX, 1> &x_pri_init,
                                       const Eigen::Matrix<double, NX, NX> &S_init,
                                       const Eigen::Matrix<double, NX, NX> &A,
                                       const Eigen::Matrix<double, NX, NU> &B,
                                       const Eigen::Matrix<double, NZ, NX> &C,
                                       const Eigen::Matrix<double, NX, NX> &R,
                                       const Eigen::Matrix<double, NZ, NZ> &Q)
    : x_pri(x_pri_init),
      S_pri(S_init),
      A(A),
      B(B),
      C(C),
      R(R),
      Q(Q) {}

template <unsigned int NX, unsigned int NU, unsigned int NZ>
void KalmanFilter<NX, NU, NZ>::update_x_pri(const Eigen::Matrix<double, NU, 1> &u)
{
  x_pri = A * x_post + B * u;
}

template <unsigned int NX, unsigned int NU, unsigned int NZ>
void KalmanFilter<NX, NU, NZ>::update_x_post(const Eigen::Matrix<double, NZ, 1> &z)
{
  x_post = x_pri + K * (z - C * x_pri);
}

template <unsigned int NX, unsigned int NU, unsigned int NZ>
void KalmanFilter<NX, NU, NZ>::update_S_pri()
{
  S_pri = A * S_post * A.transpose() + R;
}

template <unsigned int NX, unsigned int NU, unsigned int NZ>
void KalmanFilter<NX, NU, NZ>::update_S_post()
{
  S_post = (Eigen::Matrix<double, NX, NX>::Identity() - K * C) * S_pri;
}

template <unsigned int NX, unsigned int NU, unsigned int NZ>
void KalmanFilter<NX, NU, NZ>::update_K()
{
  K = S_pri * C.transpose() * (C * S_pri * C.transpose() + Q).inverse();
}

template <unsigned int NX, unsigned int NU, unsigned int NZ>
void KalmanFilter<NX, NU, NZ>::iterate(const Eigen::Matrix<double, NU, 1> &u, const Eigen::Matrix<double, NZ, 1> &z)
{
  this->update_x_pri(u);
  this->update_S_pri();
  this->update_K();
  this->update_x_post(z);
  this->update_S_post();
}

#endif // WIIMOTE_STATE_ESTIMATION_KF_H
