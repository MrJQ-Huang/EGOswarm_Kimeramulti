#include <opengv/relative_pose/CentralRelativeAdapter.hpp>
#include <opengv/relative_pose/methods.hpp>
#include <opengv/sac/Ransac.hpp>
#include <opengv/sac_problems/relative_pose/CentralRelativePoseSacProblem.hpp>
#include <Eigen/Geometry>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <numeric>
#include <random>
#include <string>

using Problem = opengv::sac_problems::relative_pose::CentralRelativePoseSacProblem;
// Inject only a deterministic sample order. Model construction, hypothesis
// rejection, iteration limits, scoring and inlier selection remain native.
class OrderedProblem : public Problem {
 public:
  OrderedProblem(adapter_t& a) : Problem(a, Problem::NISTER, false) {}
  void getSamples(int&, std::vector<int>& samples) override {
    samples = {0, 1, 2, 3, 4, 5, 6, 7};
  }
};

void run(const std::string& label, opengv::bearingVectors_t first,
         opengv::bearingVectors_t second, bool ordered,
         const Eigen::Matrix3d& truthR = Eigen::Matrix3d::Identity(),
         const Eigen::Vector3d& truthT = Eigen::Vector3d::Zero()) {
  opengv::relative_pose::CentralRelativeAdapter adapter(first, second);
  opengv::sac::Ransac<Problem> ransac;
  ransac.sac_model_ = ordered ? std::shared_ptr<Problem>(new OrderedProblem(adapter))
                            : std::shared_ptr<Problem>(new Problem(adapter, Problem::NISTER, false));
  ransac.threshold_ = 1e-8;
  ransac.max_iterations_ = 50;
  bool success = false;
  std::string exception;
  try { success = ransac.computeModel(); }
  catch (const std::exception& e) { exception = e.what(); }
  const bool finite = success && ransac.model_coefficients_.allFinite();
  double rotation_error = -1, translation_direction_error = -1;
  if (finite) {
    rotation_error = (ransac.model_coefficients_.block<3,3>(0,0) - truthR).norm();
    if (truthT.norm() > 0 && ransac.model_coefficients_.col(3).norm() > 0)
      translation_direction_error = (ransac.model_coefficients_.col(3).normalized() - truthT.normalized()).norm();
  }
  std::cout << "{\"case\":\"" << label << "\",\"success\":" << (success ? "true" : "false")
            << ",\"correspondences\":" << first.size()
            << ",\"finite_model\":" << (finite ? "true" : "false")
            << ",\"inliers\":" << ransac.inliers_.size()
            << ",\"accepted_at_20_inlier_gate\":" << (success && ransac.inliers_.size() >= 20 ? "true" : "false")
            << ",\"iterations\":" << ransac.iterations_
            << ",\"rotation_error\":" << rotation_error
            << ",\"translation_direction_error\":" << translation_direction_error
            << ",\"exception\":" << (exception.empty() ? "null" : "\"" + exception + "\"") << "}\n";
}

int main() {
  std::cout << std::setprecision(17);
  std::ifstream maps("/proc/self/maps"); std::string line;
  while (std::getline(maps, line)) {
    const auto p = line.find('/');
    if (p != std::string::npos && line.find("libopengv") != std::string::npos) {
      std::cout << "{\"case\":\"library_binding\",\"path\":\"" << line.substr(p) << "\"}\n";
      break;
    }
  }
  for (int trial=0; trial<8; ++trial) {
    std::mt19937 random(9100+trial); std::uniform_real_distribution<double> xy(-1.5,1.5), z(3.,7.);
    const Eigen::Matrix3d R = (Eigen::AngleAxisd(.06+.01*trial,Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(-.04,Eigen::Vector3d::UnitX())).toRotationMatrix();
    const Eigen::Vector3d t(.4,-.12,.07);
    opengv::bearingVectors_t first, second;
    for (int i=0; i<40; ++i) {
      const Eigen::Vector3d p(xy(random),xy(random),z(random));
      first.push_back(p.normalized()); second.push_back((R.transpose()*(p-t)).normalized());
    }
    run("normal_ransac_" + std::to_string(trial), first, second, false, R, t);
  }
  opengv::bearingVectors_t first;
  first.emplace_back(-.49090309913955377,-.098180619827910751,.86566431897438667);
  first.emplace_back(-.46188290758048278,-.092376581516096554,.88211719565622304);
  first.emplace_back(.11618577190503529,.023237154381007059,.99295563902074913);
  first.emplace_back(-.097610223533746446,-.01952204470674929,.99503323262701115);
  first.emplace_back(-.56497588229816686,-.11299517645963336,.81733367881071939);
  first.push_back(Eigen::Vector3d(.2,.04,1).normalized());
  first.push_back(Eigen::Vector3d(.4,.08,1).normalized());
  first.push_back(Eigen::Vector3d(-.3,-.06,1).normalized());
  run("exact_degenerate_ordered_full_ransac", first, first, true);
  run("same_bearings_native_sampling_full_ransac", first, first, false);
  for (int i=8; i<40; ++i)
    first.push_back(Eigen::Vector3d(-.7 + .04*i, .2*(-.7 + .04*i), 1).normalized());
  run("collinear_stationary_40_ordered_full_ransac", first, first, true);
  run("collinear_stationary_40_native_sampling_full_ransac", first, first, false);
  return 0;
}
