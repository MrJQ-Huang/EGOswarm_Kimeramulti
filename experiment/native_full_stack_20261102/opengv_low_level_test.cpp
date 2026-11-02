#include <opengv/math/Sturm.hpp>
#include <opengv/relative_pose/CentralRelativeAdapter.hpp>
#include <opengv/relative_pose/methods.hpp>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <random>
#include <string>
#include <vector>

template<class Values> void numbers(const Values& values) {
  std::cout << '[';
  for (size_t i=0;i<values.size();++i) { if(i)std::cout<<',';std::cout<<values[i]; }
  std::cout << ']';
}
template<class Matrices> void models(const Matrices& values) {
  std::cout << '[';
  for(size_t i=0;i<values.size();++i) {
    if(i)std::cout<<',';std::vector<double> row;
    for(int r=0;r<3;++r)for(int c=0;c<3;++c)row.push_back(values[i](r,c));
    numbers(row);
  }
  std::cout << ']';
}
void degenerate() {
  // Exact decimal round-trip of seed 1730 / trial 8760. Do not renormalize:
  // changing the last bit would change the floating-point degeneracy.
  opengv::bearingVectors_t first,second;
  first.emplace_back(-.49090309913955377,-.098180619827910751,.86566431897438667);
  first.emplace_back(-.46188290758048278,-.092376581516096554,.88211719565622304);
  first.emplace_back(.11618577190503529,.023237154381007059,.99295563902074913);
  first.emplace_back(-.097610223533746446,-.01952204470674929,.99503323262701115);
  first.emplace_back(-.56497588229816686,-.11299517645963336,.81733367881071939);
  second=first;opengv::relative_pose::CentralRelativeAdapter adapter(first,second);
  std::cout<<"{\"case\":\"exact_collinear_same_bearings\",";
  try {const auto result=opengv::relative_pose::fivept_nister(adapter);std::cout<<"\"models\":";models(result);std::cout<<",\"exception\":null";}
  catch(const std::exception& error){std::cout<<"\"models\":[],\"exception\":\""<<error.what()<<'"';}
  std::cout<<"}\n";
}
int main(int argc,char** argv) {
  std::cout<<std::setprecision(17);
  if(argc>1 && std::string(argv[1])=="--degenerate-only"){degenerate();return 0;}
  const std::vector<std::vector<double>> polynomials{{1,0,-1},{1,-6,11,-6},{1,0,1}};
  for(size_t i=0;i<polynomials.size();++i) {
    opengv::math::Sturm sturm(polynomials[i]);auto roots=sturm.findRoots();
    opengv::math::Sturm second(polynomials[i]);std::vector<double> roots2;second.findRoots2(roots2,1e-9,1e-11);
    std::sort(roots.begin(),roots.end());std::sort(roots2.begin(),roots2.end());
    std::cout<<"{\"case\":\"known_roots\",\"index\":"<<i<<",\"findRoots\":";numbers(roots);std::cout<<",\"findRoots2\":";numbers(roots2);std::cout<<"}\n";
  }
  for(int trial=0;trial<8;++trial) {
    std::mt19937 random(9100+trial);std::uniform_real_distribution<double> xy(-1.5,1.5),depth(3.,7.);
    const Eigen::Matrix3d rotation=(Eigen::AngleAxisd(.06+.01*trial,Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(-.04,Eigen::Vector3d::UnitX())).toRotationMatrix();
    const Eigen::Vector3d translation(.4,-.12,.07);
    Eigen::Matrix3d cross;cross<<0,-translation.z(),translation.y(),translation.z(),0,-translation.x(),-translation.y(),translation.x(),0;
    const Eigen::Matrix3d truth=cross*rotation;
    opengv::bearingVectors_t first,second;
    for(int i=0;i<5;++i){const Eigen::Vector3d point(xy(random),xy(random),depth(random));first.push_back(point.normalized());second.push_back((rotation.transpose()*(point-translation)).normalized());}
    opengv::relative_pose::CentralRelativeAdapter adapter(first,second);const auto result=opengv::relative_pose::fivept_nister(adapter);
    double best=1e100,epipolar=0.;
    for(const auto& essential:result){Eigen::Matrix3d unit=essential/essential.norm(),known=truth/truth.norm();best=std::min(best,std::min((unit-known).norm(),(unit+known).norm()));for(int i=0;i<5;++i)epipolar=std::max(epipolar,std::abs(first[i].dot(unit*second[i])));}
    std::cout<<"{\"case\":\"normal_nister\",\"index\":"<<trial<<",\"models\":";models(result);std::cout<<",\"best_true_essential_error\":"<<best<<",\"max_epipolar_residual\":"<<epipolar<<"}\n";
  }
  degenerate();return 0;
}
