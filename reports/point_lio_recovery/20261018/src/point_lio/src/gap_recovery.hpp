#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <deque>
#include <vector>
#include <cmath>
namespace gap_recovery {
struct Pose { double t; Eigen::Vector3d p; Eigen::Quaterniond q; };
struct Gap { double begin, end; };
struct Seed { Eigen::Vector3d p, vel; Eigen::Quaterniond q; };
// The aid is used only when reseeding after a detected acquisition discontinuity.
struct Aid {
    std::deque<Pose> poses;
    std::deque<Gap> gaps;
    Pose anchor_vendor, anchor_lio;
    bool anchored=false;
    double max_gap=.15, tolerance=.05;
    void add(const Pose &pose) {
        if (!std::isfinite(pose.t) || !pose.p.allFinite() || !pose.q.coeffs().allFinite() ||
            std::abs(pose.q.norm()-1.)>.05) return;
        if (!poses.empty() && pose.t<=poses.back().t) return;
        poses.push_back(pose);
        while(poses.size()>3000) poses.pop_front();
    }
    bool at(double t, Pose &out, Eigen::Vector3d *velocity=nullptr) const {
        if(poses.empty()) return false;
        auto it=std::lower_bound(poses.begin(),poses.end(),t,[](const Pose &a,double b){return a.t<b;});
        size_t right=static_cast<size_t>(it-poses.begin());
        size_t ix=std::min(right,poses.size()-1);
        if(right>0 && std::abs(poses[right-1].t-t)<std::abs(poses[ix].t-t)) ix=right-1;
        if(std::abs(poses[ix].t-t)>tolerance) return false;
        out=poses[ix];
        if(right>0 && right<poses.size() && poses[right].t-poses[right-1].t<tolerance) {
            const auto &a=poses[right-1];const auto &b=poses[right];double u=(t-a.t)/(b.t-a.t);
            out={t,a.p+u*(b.p-a.p),a.q.slerp(u,b.q).normalized()};
        }
        if(velocity) {
            size_t lo=ix,hi=ix;
            while(lo>0 && poses[ix].t-poses[lo-1].t<.06 && poses[lo].t-poses[lo-1].t<.05) --lo;
            while(hi+1<poses.size() && poses[hi+1].t-poses[ix].t<.06 && poses[hi+1].t-poses[hi].t<.05) ++hi;
            double dt=poses[hi].t-poses[lo].t;
            if(dt<.005) return false;
            *velocity=(poses[hi].p-poses[lo].p)/dt;
            if(!velocity->allFinite() || velocity->norm()>3.) return false;
        }
        return true;
    }
    bool anchor(double t,const Eigen::Vector3d &p,const Eigen::Quaterniond &q) {
        Pose vendor;
        if(!at(t,vendor)) return false;
        anchor_vendor=vendor;anchor_lio={t,p,q.normalized()};anchored=true;return true;
    }
    bool seed(double t,Seed &out) const {
        Pose vendor;Eigen::Vector3d vel;
        if(!anchored || !at(t,vendor,&vel)) return false;
        Eigen::Quaterniond map_odom=(anchor_lio.q*anchor_vendor.q.conjugate()).normalized();
        out.q=(map_odom*vendor.q).normalized();
        out.p=anchor_lio.p+map_odom*(vendor.p-anchor_vendor.p);
        out.vel=map_odom*vel;
        return out.p.allFinite() && out.vel.allFinite();
    }
};
}
