//
// Created by lfc on 2021/2/28.
//


//ros lib
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>

//pcl lib
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

//gazebo lib
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/MultiRayShape.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/transport/Node.hh>

//local lib
#include "livox_laser_simulation/csv_reader.hpp"
#include "livox_laser_simulation/livox_points_plugin.h"
#include "livox_laser_simulation/livox_ode_multiray_shape.h"

namespace gazebo {

GZ_REGISTER_SENSOR_PLUGIN(LivoxPointsPlugin)

LivoxPointsPlugin::LivoxPointsPlugin() {}

LivoxPointsPlugin::~LivoxPointsPlugin() {}

void convertDataToRotateInfo(const std::vector<std::vector<double>> &datas, std::vector<AviaRotateInfo> &avia_infos) {
    avia_infos.reserve(datas.size());
    double deg_2_rad = M_PI / 180.0;
    for (auto &data : datas) {
        if (data.size() == 3) {
            avia_infos.emplace_back();
            avia_infos.back().time = data[0];
            avia_infos.back().azimuth = data[1] * deg_2_rad;
            avia_infos.back().zenith = data[2] * deg_2_rad - M_PI_2;  //转化成标准的右手系角度
        } else {
            ROS_INFO_STREAM("data size is not 3!");
        }
    }
}

void LivoxPointsPlugin::Load(gazebo::sensors::SensorPtr _parent, sdf::ElementPtr _sdf) {
    RayPlugin::Load(_parent, _sdf);

    // initialize gazebo node 
    node = transport::NodePtr(new transport::Node());
    node->Init(_parent->WorldName());

    // initialize ROS node 
    int argc = 0;
    char **argv = nullptr;
    ros::init(argc, argv, _parent->Name());
    rosNode.reset(new ros::NodeHandle);

    // load scan pattern description csv file 
    std::vector<std::vector<double>> datas;
    std::string file_name = _sdf->Get<std::string>("csv_file_name");
    ROS_INFO_STREAM("load csv file name:" << file_name);
    if (!CsvReader::ReadCsvFile(file_name, datas)) {
        ROS_FATAL_STREAM("cannot get csv file : " << file_name);
        return;
    }
    aviaInfos.clear();
    convertDataToRotateInfo(datas, aviaInfos);

    raySensor = _parent;
    parentEntity = this->world->EntityByName(raySensor->ParentName());

    // read parameters from sdf
    sdfPtr = _sdf;
    auto rayElem = sdfPtr->GetElement("ray");
    auto scanElem = rayElem->GetElement("scan");
    auto rangeElem = rayElem->GetElement("range");
    minDist = rangeElem->Get<double>("min");
    maxDist = rangeElem->Get<double>("max");
    ROS_DEBUG_STREAM("scan range :" << minDist << " - " << maxDist);
    auto curr_scan_topic = sdfPtr->Get<std::string>("ros_topic");
    ROS_DEBUG_STREAM("ros topic name:" << curr_scan_topic);
    samplesStep = sdfPtr->Get<int>("samples");
    ROS_DEBUG_STREAM("sample:" << samplesStep);
    downSample = std::min(sdfPtr->Get<int>("downsample"), 1);
    ROS_DEBUG_STREAM("downsample:" << downSample);
    maxPointSize = aviaInfos.size();
    ROS_DEBUG_STREAM("scan info size:" << aviaInfos.size());

    // gazebo scan publisher
    scanPub = node->Advertise<msgs::LaserScanStamped>(raySensor->Topic(), 50);

    // ros scan publisher
    rosPointPub = rosNode->advertise<sensor_msgs::PointCloud2>(curr_scan_topic, 5);

    // create the laser collisions calculator
    laserCollision = world->Physics()->CreateCollision("multiray", raySensor->ParentName());
    laserCollision->SetName("ray_sensor_collision");
    laserCollision->SetRelativePose(raySensor->Pose());
    laserCollision->SetInitialRelativePose(raySensor->Pose());
    rayShape.reset(new gazebo::physics::LivoxOdeMultiRayShape(laserCollision));
    laserCollision->SetShape(rayShape);
    rayShape->RayShapes().reserve(samplesStep);
    rayShape->Load(sdfPtr);
    // rayShape->Init();
}

void LivoxPointsPlugin::OnNewLaserScans() {
    RayPlugin::OnNewLaserScans();

    if (!rayShape) 
        return; // can't do anything if rayShape has not been initialized yet

    sensor_msgs::PointCloud2::Ptr ros_scan_point(new sensor_msgs::PointCloud2);

    // update ray positions
    InitializeRays();

    // update ray collisions
    rayShape->Update();

    // place collision points from the updated rays into pointcloud
    RetrieveCollisionPointCloud(ros_scan_point);

    // prepare gazebo message for visualization
    // InitializeScan();

    // publish gazebo msg for visualization
    // Note : this makes everything slow AF
    // if (scanPub && scanPub->HasConnections()) 
    //     scanPub->Publish(laserMsg);

    // publish ros message
    if (rosPointPub && rosPointPub.getNumSubscribers() > 0) 
        rosPointPub.publish(ros_scan_point);

    // spin once for callback handling
    ros::spinOnce();
}

void LivoxPointsPlugin::InitializeRays() {
    bool create_rays = (rayShape->RayCount() < samplesStep);
    ignition::math::Vector3d start_point, 
                             end_point,
                             axis, 
                             unit_x(1.0, 0.0, 0.0);
    ignition::math::Quaterniond ray;

    auto offset = laserCollision->RelativePose();

    // update ray positions
    for (int i = 0; i < samplesStep; i++) {
        auto &rotate_info = aviaInfos[(currStartIndex + i) % maxPointSize];

        // calculated the ray axis
        ray.Euler(0.0, rotate_info.zenith, rotate_info.azimuth);
        axis = offset.Rot() * ray * unit_x;

        // start & end points of the ray
        start_point = minDist * axis + offset.Pos();
        end_point = maxDist * axis + offset.Pos();

        // create new rays / update ray info
        if (create_rays) 
            rayShape->AddRay(start_point, end_point);
        else 
            rayShape->SetRay(i, start_point, end_point);
    }

    // update start index for next iteration
    currStartIndex += samplesStep;
}

void LivoxPointsPlugin::InitializeScan() {

    msgs::LaserScan* scan = laserMsg.mutable_scan();
    bool create_rays = (scan->count() !=  rayShape->RayCount());
    
    // Store the latest laser scans into laserMsg
    scan->set_frame(raySensor->Name());
    msgs::Set(scan->mutable_world_pose(), raySensor->Pose() + parentEntity->WorldPose());
    msgs::Set(laserMsg.mutable_time(), world->SimTime());

    scan->set_angle_min(AngleMin().Radian());
    scan->set_angle_max(AngleMax().Radian());
    scan->set_angle_step(AngleResolution());
    scan->set_count(RangeCount());

    scan->set_vertical_angle_min(VerticalAngleMin().Radian());
    scan->set_vertical_angle_max(VerticalAngleMax().Radian());
    scan->set_vertical_angle_step(VerticalAngleResolution());
    scan->set_vertical_count(VerticalRangeCount());

    scan->set_range_min(RangeMin());
    scan->set_range_max(RangeMax());

    if (create_rays) {
        scan->clear_ranges();
        scan->clear_intensities();

        for (int i = 0; i < rayShape->RayCount(); i++) {
            scan->add_ranges(rayShape->GetRange(i));
            scan->add_intensities(rayShape->GetRetro(i));
        }
    }
    else {
        for (int i = 0; i < rayShape->RayCount(); i++) {
            scan->set_ranges(i, rayShape->GetRange(i));
            scan->set_intensities(i, rayShape->GetRetro(i));
        }
    }

}

void LivoxPointsPlugin::RetrieveCollisionPointCloud(sensor_msgs::PointCloud2::Ptr point_cloud){
    pcl::PointXYZI pcl_point; 
    pcl::PointCloud<pcl::PointXYZI>::Ptr scan_points(new pcl::PointCloud<pcl::PointXYZI>());

    static int seq_cnt = 0; // pointcloud message counter
    auto offset = laserCollision->RelativePose();
    for (int i = 0; i < rayShape->RayCount(); i++) {

        if (rayShape->GetRange(i) < RangeMax()) {
            auto point = rayShape->Ray(i)->End() - offset.Pos();
            pcl_point.x = point.X();
            pcl_point.y = point.Y();
            pcl_point.z = point.Z();
        }
        else {
            pcl_point.x = std::nan("");
            pcl_point.y = std::nan("");
            pcl_point.z = std::nan("");
        }

        auto intensity = rayShape->GetRetro(i);
        pcl_point.intensity = intensity;

        scan_points->points.push_back(pcl_point);
    }
    scan_points->header.stamp = ros::Time::now().toSec()+1e6;
    scan_points->header.seq = seq_cnt++;
    scan_points->header.frame_id = raySensor->Name();

    pcl::toROSMsg(*scan_points, *point_cloud);
}

ignition::math::Angle LivoxPointsPlugin::AngleMin() const {
    if (rayShape)
        return rayShape->MinAngle();
    else
        return -1;
}

ignition::math::Angle LivoxPointsPlugin::AngleMax() const {
    if (rayShape) {
        return ignition::math::Angle(rayShape->MaxAngle().Radian());
    } else
        return -1;
}

double LivoxPointsPlugin::GetRangeMin() const { return RangeMin(); }

double LivoxPointsPlugin::RangeMin() const {
    if (rayShape)
        return rayShape->GetMinRange(); //
    else
        return -1;
}

double LivoxPointsPlugin::GetRangeMax() const { return RangeMax(); }

double LivoxPointsPlugin::RangeMax() const {
    if (rayShape)
        return rayShape->GetMaxRange();
    else
        return -1;
}

double LivoxPointsPlugin::GetAngleResolution() const { return AngleResolution(); }

double LivoxPointsPlugin::AngleResolution() const { 
    return (AngleMax() - AngleMin()).Radian() / (RangeCount() - 1); 
}

double LivoxPointsPlugin::GetRangeResolution() const { return RangeResolution(); }

double LivoxPointsPlugin::RangeResolution() const {
    if (rayShape)
        return rayShape->GetResRange();
    else
        return -1;
}

int LivoxPointsPlugin::GetRayCount() const { return RayCount(); }

int LivoxPointsPlugin::RayCount() const {
    if (rayShape)
        return rayShape->GetSampleCount();
    else
        return -1;
}

int LivoxPointsPlugin::GetRangeCount() const { return RangeCount(); }

int LivoxPointsPlugin::RangeCount() const {
    if (rayShape)
        return rayShape->GetSampleCount() * rayShape->GetScanResolution();
    else
        return -1;
}

int LivoxPointsPlugin::GetVerticalRayCount() const { return VerticalRayCount(); }

int LivoxPointsPlugin::VerticalRayCount() const {
    if (rayShape)
        return rayShape->GetVerticalSampleCount();
    else
        return -1;
}

int LivoxPointsPlugin::GetVerticalRangeCount() const { return VerticalRangeCount(); }

int LivoxPointsPlugin::VerticalRangeCount() const {
    if (rayShape)
        return rayShape->GetVerticalSampleCount() * rayShape->GetVerticalScanResolution();
    else
        return -1;
}

ignition::math::Angle LivoxPointsPlugin::VerticalAngleMin() const {
    if (rayShape) {
        return ignition::math::Angle(rayShape->VerticalMinAngle().Radian());
    } else
        return -1;
}

ignition::math::Angle LivoxPointsPlugin::VerticalAngleMax() const {
    if (rayShape) {
        return ignition::math::Angle(rayShape->VerticalMaxAngle().Radian());
    } else
        return -1;
}

double LivoxPointsPlugin::GetVerticalAngleResolution() const { return VerticalAngleResolution(); }

double LivoxPointsPlugin::VerticalAngleResolution() const {
    return (VerticalAngleMax() - VerticalAngleMin()).Radian() / (VerticalRangeCount() - 1);
}

void LivoxPointsPlugin::SendRosTf(const ignition::math::Pose3d &pose, const std::string &father_frame,
                                  const std::string &child_frame) {
    if (!tfBroadcaster) {
        tfBroadcaster.reset(new tf::TransformBroadcaster);
    }
    tf::Transform tf;
    auto rot = pose.Rot();
    auto pos = pose.Pos();
    tf.setRotation(tf::Quaternion(rot.X(), rot.Y(), rot.Z(), rot.W()));
    tf.setOrigin(tf::Vector3(pos.X(), pos.Y(), pos.Z()));
    tfBroadcaster->sendTransform(
        tf::StampedTransform(tf, ros::Time::now(), father_frame, child_frame));
}

}
