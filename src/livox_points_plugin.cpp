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

void LivoxPointsPlugin::Load(gazebo::sensors::SensorPtr _parent, sdf::ElementPtr sdf) {

    // initialize gazebo node 
    node = transport::NodePtr(new transport::Node());
    node->Init(_parent->WorldName());

    // initialize ROS node 
    int argc = 0;
    char **argv = nullptr;
    ros::init(argc, argv, _parent->Name());
    rosNode.reset(new ros::NodeHandle);

    // initialize plugin 
    RayPlugin::Load(_parent, sdf);

    // load scan pattern description csv file 
    std::vector<std::vector<double>> datas;
    std::string file_name = sdf->Get<std::string>("csv_file_name");
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
    sdfPtr = sdf;
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
    laserMsg.mutable_scan()->set_frame(raySensor->Name());

    // ros scan publisher
    rosPointPub = rosNode->advertise<sensor_msgs::PointCloud2>(curr_scan_topic, 5);

    // send TF between sensor and base 
    // Note : don't do this because it's cheating 
    // SendRosTf(raySensor->Pose(), parentEntity->GetName(), raySensor->Name());

    // create the laser collisions calulator
    laserCollision = world->Physics()->CreateCollision("multiray", raySensor->ParentName());
    laserCollision->SetName("ray_sensor_collision");
    laserCollision->SetRelativePose(raySensor->Pose());
    laserCollision->SetInitialRelativePose(raySensor->Pose());
    rayShape.reset(new gazebo::physics::LivoxOdeMultiRayShape(laserCollision));
    laserCollision->SetShape(rayShape);
    rayShape->RayShapes().reserve(samplesStep / downSample);
    rayShape->Load(sdfPtr);
    rayShape->Init();

    ignition::math::Vector3d start_point, end_point;
    ignition::math::Quaterniond ray;
    for (int j = 0; j < samplesStep; j += downSample) {
        int index = j % maxPointSize;
        auto &rotate_info = aviaInfos[index];
        ray.Euler(ignition::math::Vector3d(0.0, rotate_info.zenith, rotate_info.azimuth));
        auto axis = laserCollision->RelativePose().Rot() * ray * ignition::math::Vector3d(1.0, 0.0, 0.0);
        start_point = minDist * axis + laserCollision->RelativePose().Pos();
        end_point = maxDist * axis + laserCollision->RelativePose().Pos();
        rayShape->AddRay(start_point, end_point);
    }

    msgs::LaserScan *scan = laserMsg.mutable_scan();
    InitializeScan(scan);
}

void LivoxPointsPlugin::OnNewLaserScans() {
    if (!rayShape) 
        return; // can't do anything if rayShape has not been initialized yet

    std::vector<std::pair<int, AviaRotateInfo>> points_pair;
    InitializeRays(points_pair, rayShape);
    rayShape->Update();

    // update time
    msgs::Set(laserMsg.mutable_time(), world->SimTime());


    // send TF between sensor and base 
    // Note : don't do this because it's cheating 
    // SendRosTf(raySensor->Pose(), parentEntity->GetName(), raySensor->Name());
    // SendRosTf(parentEntity->WorldPose(), world->Name(), raySensor->Name());

    static int seq_cnt = 0;
    pcl::PointXYZI pcl_point; 
    pcl::PointCloud<pcl::PointXYZI>::Ptr scan_points(new pcl::PointCloud<pcl::PointXYZI>());
    scan_points->header.stamp = laserMsg.time().sec()*1000 + laserMsg.time().nsec()/1e6;
    scan_points->header.seq = seq_cnt++;
    scan_points->header.frame_id = laserMsg.scan().frame();

    for (auto &pair : points_pair) {

        auto range = rayShape->GetRange(pair.first);
        auto intensity = rayShape->GetRetro(pair.first);
        auto rotate_info = pair.second;


        if (range < RangeMin() || range > RangeMax()) 
            range = std::nan("1");

        ignition::math::Quaterniond ray;
        ray.Euler(ignition::math::Vector3d(0.0, rotate_info.zenith, rotate_info.azimuth));

        auto axis = ray * ignition::math::Vector3d(1.0, 0.0, 0.0);
        auto point = range * axis;
        pcl_point.x = point.X();
        pcl_point.y = point.Y();
        pcl_point.z = point.Z();
        pcl_point.intensity = intensity;

        scan_points->points.push_back(pcl_point);
    }

    // publish gazebo msg
    if (scanPub && scanPub->HasConnections()) 
        scanPub->Publish(laserMsg);

    // publish ros message
    sensor_msgs::PointCloud2 ros_scan_point;
    pcl::toROSMsg(*scan_points, ros_scan_point);
    rosPointPub.publish(ros_scan_point);

    // spin once for callback handling
    ros::spinOnce();
}

void LivoxPointsPlugin::InitializeRays(std::vector<std::pair<int, AviaRotateInfo>> &points_pair,
                                       boost::shared_ptr<physics::LivoxOdeMultiRayShape> &ray_shape) {
    auto &rays = ray_shape->RayShapes();
    ignition::math::Vector3d start_point, end_point;
    ignition::math::Quaterniond ray;
    auto offset = laserCollision->RelativePose();
    int64_t end_index = currStartIndex + samplesStep;
    auto ray_size = rays.size();
    points_pair.reserve(rays.size());
    for (int k = currStartIndex, ray_index = 0; k < end_index && ray_index < ray_size; k += downSample, ray_index++) {
        auto index = k % maxPointSize;
        auto &rotate_info = aviaInfos[index];
        ray.Euler(ignition::math::Vector3d(0.0, rotate_info.zenith, rotate_info.azimuth));
        auto axis = offset.Rot() * ray * ignition::math::Vector3d(1.0, 0.0, 0.0);
        start_point = minDist * axis + offset.Pos();
        end_point = maxDist * axis + offset.Pos();
        rays[ray_index]->SetPoints(start_point, end_point);
        points_pair.emplace_back(ray_index, rotate_info);
    }
    currStartIndex += samplesStep;
}

void LivoxPointsPlugin::InitializeScan(msgs::LaserScan *&scan) {
    // Store the latest laser scans into laserMsg
    msgs::Set(scan->mutable_world_pose(), raySensor->Pose() + parentEntity->WorldPose());
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

    scan->clear_ranges();
    scan->clear_intensities();

    for (unsigned int j = 0; j < scan->vertical_count(); ++j) {
        for (unsigned int i = 0; i < scan->count(); ++i) {
            scan->add_ranges(0);
            scan->add_intensities(0);
        }
    }
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

double LivoxPointsPlugin::AngleResolution() const { return (AngleMax() - AngleMin()).Radian() / (RangeCount() - 1); }

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
