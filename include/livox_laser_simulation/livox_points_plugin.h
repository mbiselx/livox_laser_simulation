//
// Created by lfc on 2021/2/28.
//

#ifndef SRC_GAZEBO_LIVOX_POINTS_PLUGIN_H
#define SRC_GAZEBO_LIVOX_POINTS_PLUGIN_H

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

#include <gazebo/transport/Node.hh>
#include <gazebo/plugins/RayPlugin.hh>
#include <gazebo/physics/MultiRayShape.hh>
#include <gazebo/sensors/Noise.hh>

namespace gazebo {

struct RayRotationInfo {
    double time;
    double azimuth;
    double zenith;
};

class LivoxPointsPlugin : public RayPlugin {
 public:
    LivoxPointsPlugin();

    virtual ~LivoxPointsPlugin();

    void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf); 


   protected:
      void ConnectCb();

      void OnScan(ConstLaserScanStampedPtr& _msg);

      // void OnNewLaserScans();

      void UpdateRays();

   private: 
      // parent ray sensor 
      sensors::RaySensorPtr parent_sensor;

      // sdf element related to this plugin
      sdf::ElementPtr sdfPtr;

      // gazebo node handle
      transport::NodePtr gz_node;

      // ROS node handle
      ros::NodeHandle* ros_node;

      // gazebo laser scan subscriber
      transport::SubscriberPtr gz_sub;

      // ROS pointcloud publisher
      ros::Publisher ros_pub;

      // ROS topic on which to publish the pointcloud
      std::string topic_name;

      // frame of the ROS pointcloud
      std::string frame_name;

      // container for rays
      physics::MultiRayShapePtr ray_shape;
      std::vector<RayRotationInfo> ray_rotations;

      // Custom Callback Queue
      ros::CallbackQueue laser_queue_;
      void laserQueueThread();
      boost::thread callback_laser_queue_thread_;

      int64_t samples_step = 0;
      int64_t current_samples_idx = 0;

};

}  // namespace gazebo

#endif  // SRC_GAZEBO_LIVOX_POINTS_PLUGIN_H
