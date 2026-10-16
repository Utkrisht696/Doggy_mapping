#!/usr/bin/env python3
"""Remove invalid LiDAR point records; retain source frames, stamps and fields."""
import array
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import PointCloud2
from lidar_cloud_utils import cloud_arrays


class CleanClouds(Node):
    def __init__(self):
        super().__init__('doggy_clean_lidar_clouds')
        self.stats={}
        self.last_stamp={}
        self.subs=[]
        self.pubs={}
        self.sources={
            '/utlidar/cloud':('/doggy/lidar/cloud_raw','utlidar_lidar'),
            '/utlidar/cloud_base':('/doggy/lidar/cloud_body','base_link'),
            '/utlidar/cloud_deskewed':('/doggy/lidar/cloud_deskewed','odom')}
        for source,(output,frame) in self.sources.items():
            self.pubs[source]=self.create_publisher(PointCloud2,output,qos_profile_sensor_data)
            self.subs.append(self.create_subscription(PointCloud2,source,lambda msg,s=source:self.receive(s,msg),qos_profile_sensor_data))
        self.create_timer(5.0,self.status)
        self.get_logger().info('Filtering zero/nonfinite XYZ only; source coordinates, timestamps and point fields retained. No SLAM or commands.')

    def receive(self,source,msg):
        processing_started=time.perf_counter()
        output,expected=self.sources[source]
        if msg.header.frame_id!=expected:
            self.get_logger().error(f'{source}: unexpected frame {msg.header.frame_id}; not publishing')
            return
        stamp=msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
        if stamp<=0:
            self.get_logger().error(f'{source}: invalid timestamp; not publishing')
            return
        try:
            xyz,records,valid=cloud_arrays(msg)
        except (ValueError,TypeError) as exc:
            self.get_logger().error(f'{source}: {exc}')
            return
        clean=PointCloud2()
        clean.header=msg.header
        clean.height=1
        clean.width=int(valid.sum())
        clean.fields=msg.fields
        clean.is_bigendian=msg.is_bigendian
        clean.point_step=msg.point_step
        clean.row_step=clean.width*clean.point_step
        # Humble's bytes setter validates each byte in Python; a typed array
        # takes the fast path and avoids building up a sensor callback queue.
        clean.data=array.array('B',records[valid].tobytes())
        clean.is_dense=True
        self.pubs[source].publish(clean)
        row=self.stats.setdefault(source,{'messages':0,'removed':0,'time_backsteps':0,'max_processing_ms':0})
        if source in self.last_stamp and stamp<self.last_stamp[source]:
            row['time_backsteps']+=1
            self.get_logger().warning(f'{source}: source clock moved backward; original timestamp retained')
        self.last_stamp[source]=stamp
        row['messages']+=1
        row['removed']+=int(len(valid)-valid.sum())
        row['last_usable_points']=clean.width
        row['max_processing_ms']=max(row['max_processing_ms'],(time.perf_counter()-processing_started)*1000)

    def status(self):
        for source,row in self.stats.items():
            self.get_logger().info(f'{source}: {row}')


def main():
    rclpy.init()
    node=CleanClouds()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        # ros2 launch may forward SIGINT after the terminal's group signal.
        import signal
        signal.signal(signal.SIGINT,signal.SIG_IGN)
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
