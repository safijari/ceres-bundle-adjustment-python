#include <iostream>
#include <fstream>
#include <vector>
#include "ceres/ceres.h"
#include "ceres/rotation.h"

// Define the cost function for bundle adjustment
struct ReprojectionError {
  ReprojectionError(double observed_x, double observed_y)
    : observed_x(observed_x), observed_y(observed_y) {}

  template <typename T>
  bool operator()(const T* const camera,
                  const T* const point,
                  T* residuals) const {
    // Rotate point to camera frame
    T p[3];
    ceres::AngleAxisRotatePoint(camera, point, p);
    // Translate point to camera frame
    p[0] += camera[3];
    p[1] += camera[4];
    p[2] += camera[5];

    // Project onto image plane
    T xp = p[0] / p[2];
    T yp = p[1] / p[2];

    // Compute final projected point position
    T predicted_x = camera[6] * xp + camera[8];
    T predicted_y = camera[7] * yp + camera[9];

    // T predicted_x = 385.81 * xp + 322.427;
    // T predicted_y = 385.81 * yp + 240.169;

    // Calculate residuals (difference between observed and predicted)
    residuals[0] = predicted_x - T(observed_x);
    residuals[1] = predicted_y - T(observed_y);

    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(const double observed_x,
                                     const double observed_y) {
    return new ceres::AutoDiffCostFunction<ReprojectionError, 2, 6, 3>
      (new ReprojectionError(observed_x, observed_y));
  }

  double observed_x;
  double observed_y;
};

typedef std::vector<std::vector<double> > vecvecd;

std::pair<vecvecd, vecvecd> do_bundle_adjustment(vecvecd &camera_params,
                          vecvecd &point_params,
                          vecvecd &observations) {
  // each observation contains point and camera index at position 2 and 3
  ceres::Problem problem;
  for (size_t i = 0; i < observations.size(); i++) {
    ceres::CostFunction* cost_function =
      ReprojectionError::Create(observations[i][0], observations[i][1]);
    problem.AddResidualBlock(cost_function, nullptr,
                             camera_params[observations[i][3]].data(),
                             point_params[observations[i][2]].data());
  }
  // Configure options for Ceres Solver
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::SPARSE_SCHUR;
  options.minimizer_progress_to_stdout = true;
  // Solve the problem
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  // std::cout << summary.FullReport() << std::endl;
  return std::pair<vecvecd, vecvecd>(camera_params, point_params);
}

// pybind11 code to expose the do_bundle_adjustment function to python

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

PYBIND11_MODULE(ceres_bundle_adjustment, m) {
    m.doc() = "Bundle Adjustment using Ceres Solver";
    m.def("do_bundle_adjustment", &do_bundle_adjustment, "Perform bundle adjustment using Ceres Solver");
}