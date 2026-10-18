#include "src/point_lio/src/gap_recovery.hpp"
#include <cassert>
#include <iostream>
using namespace gap_recovery;
int main(){
 Aid a;Seed s;assert(!a.seed(1.,s));
 Eigen::Quaterniond turn(Eigen::AngleAxisd(1.5707963267948966,Eigen::Vector3d::UnitZ()));
 for(int i=0;i<20;++i)a.add({.01*i,Eigen::Vector3d(.01*i,0,0),Eigen::Quaterniond::Identity()});
 assert(a.anchor(.1,Eigen::Vector3d(10,20,3),turn));
 for(int i=0;i<20;++i)a.add({4.+.01*i,Eigen::Vector3d(1.+.01*i,2,0),turn});
 assert(!a.seed(2.,s)); // no interpolation across missing acquisition time
 assert(a.seed(4.1,s));
 assert((s.p-Eigen::Vector3d(8.,21.,3.)).norm()<1e-10);
 assert(std::abs(s.q.angularDistance(turn*turn))<1e-10);
 assert((s.vel-Eigen::Vector3d(0,1,0)).norm()<1e-10);
 Pose p;assert(!a.at(8.,p)); // stale vendor pose is rejected
 auto n=a.poses.size();a.add({4.1,Eigen::Vector3d::Zero(),turn});assert(a.poses.size()==n);
 Aid b;b.add({1,Eigen::Vector3d::Zero(),turn});assert(b.anchor(1,Eigen::Vector3d::Zero(),turn));assert(!b.seed(1,s)); // pose without enough velocity support cannot seed
 std::cout<<"PASS: transformed relative pose, velocity, gap/stale/duplicate/missing-aid guards\n";
}
