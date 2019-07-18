#ifndef WIIMOTE_STATE_ESTIMATION_FILTER_METHOD_H
#define WIIMOTE_STATE_ESTIMATION_FILTER_METHOD_H

#include <Eigen/Dense>

template <unsigned int NX, unsigned int NU, unsigned int NZ>
class FilterMethod {
public:
	/**
   * Get current a posteriori state.
   * @return const Eigen::Matrix<double, NX, 1>& Constant reference to the
   * internal x_post.
   */
	virtual const Eigen::Matrix<double, NX, 1> &getState() const = 0;

	/**
   * Get current a posteriori covariance matrix.
   * @return const Eigen::Matrix<double, NX, 1>& Constant reference to the
   * internal S_post.
   */
	virtual const Eigen::Matrix<double, NX, NX> &getCoVar() const = 0;

	/**
   * Run one iteration of the algorithm, given a current input and measurement.
   * @param u Current input.
   * @param z Current measurement.
   */
	virtual void iterate(const Eigen::Matrix<double, NU, 1> &u,
						 const Eigen::Matrix<double, NZ, 1> &z) = 0;

	virtual ~FilterMethod() {}
};

#endif // WIIMOTE_STATE_ESTIMATION_FILTER_METHOD_H