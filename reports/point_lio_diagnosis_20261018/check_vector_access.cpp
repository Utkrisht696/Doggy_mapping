#include <Eigen/Core>
#include <vector>
#include <string>
#include <iostream>
int main(int argc,char**argv) {
 std::vector<Eigen::Vector3d> points;
 std::vector<Eigen::Matrix3d> crosses;
 bool corrected=argc>1 && std::string(argv[1])=="resize";
 if(corrected) { points.resize(2);crosses.resize(2); }
 else { points.reserve(2);crosses.reserve(2); }
 std::cout<<"points size="<<points.size()<<" capacity="<<points.capacity()<<std::endl;
 points[0]=Eigen::Vector3d(1.,2.,3.);
 crosses[0]=Eigen::Matrix3d::Identity();
 return points[0].isApprox(Eigen::Vector3d(1.,2.,3.)) && crosses[0].isIdentity() ? 0:1;
}
