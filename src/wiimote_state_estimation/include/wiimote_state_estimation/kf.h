#ifndef WIIMOTE_STATE_ESTIMATION_KF_H
#define WIIMOTE_STATE_ESTIMATION_KF_H

#include <Eigen/Sparse>

/**
 * Template class that uses the linear Kalman Filter algorithm to filter noisy measurement data.
 * @param NX Number of states
 * @param NU Number of inputs
 * @param NZ Number of measurements
 */
template <unsigned int NX, unsigned int NU, unsigned int NZ>
class KalmanFilter
{
public:
  KalmanFilter(const Eigen::Matrix<double, NX, 1> &x_pri_init,
               const Eigen::Matrix<double, NX, NX> &S_init,
               const Eigen::Matrix<double, NX, NX> &A,
               const Eigen::Matrix<double, NX, NU> &B,
               const Eigen::Matrix<double, NZ, NX> &C,
               const Eigen::Matrix<double, NX, NX> &R_init,
               const Eigen::Matrix<double, NZ, NZ> &Q_init);
  void iterate();
  Eigen::Matrix<double, NX, 1> getState() const;

private:
  Eigen::Matrix<double, NX, 1> x_pri;
  Eigen::Matrix<double, NX, 1> x_post;
  Eigen::Matrix<double, NX, NX> S;
  Eigen::Matrix<double, NX, NX> A;
  Eigen::Matrix<double, NX, NU> B;
  Eigen::Matrix<double, NZ, NX> C;
  Eigen::Matrix<double, NX, NZ> K;
  Eigen::Matrix<double, NX, NX> R;
  Eigen::Matrix<double, NZ, NZ> Q;
  void update_x_pri();
};

template <unsigned int NX, unsigned int NU, unsigned int NZ>
KalmanFilter<NX, NU, NZ>::KalmanFilter(const Eigen::Matrix<double, NX, 1> &x_pri_init,
                                       const Eigen::Matrix<double, NX, NX> &S_init,
                                       const Eigen::Matrix<double, NX, NX> &A,
                                       const Eigen::Matrix<double, NX, NU> &B,
                                       const Eigen::Matrix<double, NZ, NX> &C,
                                       const Eigen::Matrix<double, NX, NX> &R_init,
                                       const Eigen::Matrix<double, NZ, NZ> &Q_init)
    : x_pri(x_pri_init),
      S(S_init),
      A(A),
      B(B),
      C(C),
      R(R_init),
      Q(Q_init) {}

#endif // WIIMOTE_STATE_ESTIMATION_KF_H
