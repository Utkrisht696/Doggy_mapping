// Temporary read-only diagnostic. No writer, service client or motion API.
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/ros2/Imu_.hpp>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <thread>
#include <chrono>
#include <iostream>
int main(int argc,char**argv) {
  if(argc!=2) return 2;
  std::ofstream out(argv[1]);
  if(!out) return 3;
  out << "sec,nsec,frame,qx,qy,qz,qw,gx,gy,gz,ax,ay,az";
  for(int j=0;j<27;++j) out << ",cov" << j;
  out << '\n' << std::setprecision(17);
  std::mutex guard;
  size_t count=0;
  try {
    unitree::robot::ChannelFactory::Instance()->Init(0,"enp7s0");
    unitree::robot::ChannelSubscriber<sensor_msgs::msg::dds_::Imu_> sub("rt/utlidar/imu");
    sub.InitChannel([&](const void* ptr) {
      const auto &m=*static_cast<const sensor_msgs::msg::dds_::Imu_*>(ptr);
      std::lock_guard<std::mutex> lock(guard);
      out<<m.header().stamp().sec()<<','<<m.header().stamp().nanosec()<<','<<m.header().frame_id()
         <<','<<m.orientation().x()<<','<<m.orientation().y()<<','<<m.orientation().z()<<','<<m.orientation().w()
         <<','<<m.angular_velocity().x()<<','<<m.angular_velocity().y()<<','<<m.angular_velocity().z()
         <<','<<m.linear_acceleration().x()<<','<<m.linear_acceleration().y()<<','<<m.linear_acceleration().z();
      for(auto v:m.orientation_covariance()) out<<','<<v;
      for(auto v:m.angular_velocity_covariance()) out<<','<<v;
      for(auto v:m.linear_acceleration_covariance()) out<<','<<v;
      out<<'\n'; ++count;
    });
    std::this_thread::sleep_for(std::chrono::seconds(14));
    sub.CloseChannel();
    std::lock_guard<std::mutex> lock(guard);
    out.flush(); std::cout<<"Native SDK IMU messages: "<<count<<std::endl;
    return count?0:4;
  } catch(const std::exception &e) {std::cerr<<e.what()<<std::endl;return 5;}
}
