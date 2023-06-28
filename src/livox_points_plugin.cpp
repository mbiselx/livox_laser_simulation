//
// Created by lfc on 2021/2/28.
//


//ros lib
#include <sensor_msgs/PointCloud2.h>

//local lib
#include "livox_laser_simulation/csv_reader.hpp"
#include "livox_laser_simulation/livox_points_plugin.h"
#include "livox_laser_simulation/livox_ode_multiray_shape.h"

#define POINTFIELD_X 0
#define POINTFIELD_Y 1
#define POINTFIELD_Z 2
#define POINTFIELD_DST 3
#define POINTFIELD_INT 4
#define POINTFIELD_CHN 5

using namespace gazebo;


void convertDataToRotateInfo(const std::vector<std::vector<double>> &datas, std::vector<RayRotationInfo> &avia_infos) {
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


GZ_REGISTER_SENSOR_PLUGIN(LivoxPointsPlugin)

LivoxPointsPlugin::LivoxPointsPlugin() {}

LivoxPointsPlugin::~LivoxPointsPlugin() {}

void LivoxPointsPlugin::Load(gazebo::sensors::SensorPtr _parent, sdf::ElementPtr _sdf) {
    RayPlugin::Load(_parent, _sdf); // super

    this->sdfPtr = _sdf;
    this->parent_sensor = std::dynamic_pointer_cast<sensors::RaySensor>(_parent);
    this->ray_shape = this->parent_sensor->LaserShape();

    // initialize gazebo node 
    this->gz_node = transport::NodePtr(new transport::Node());
    this->gz_node->Init();

    // initialize ROS ros_node 
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = nullptr;
        ros::init(argc, nullptr, this->parent_sensor->Name());
    }
    this->ros_node = new ros::NodeHandle("/");

    // load scan pattern description csv file 
    std::vector<std::vector<double>> data;
    std::string file_name = _sdf->Get<std::string>("csv_file_name");
    ROS_INFO_STREAM("load csv file name:" << file_name);
    if (!CsvReader::ReadCsvFile(file_name, data)) {
        ROS_FATAL_STREAM("cannot get csv file : " << file_name);
        return;
    }
    ray_rotations.clear();
    convertDataToRotateInfo(data, ray_rotations);

    // load SDF values
    this->samples_step = this->sdfPtr->Get<int>("samples");
    ROS_INFO_STREAM("samples : " << this->samples_step);

    this->topic_name = this->sdfPtr->Get<std::string>("topicName"); 
    this->frame_name = this->sdfPtr->Get<std::string>("frameName"); 
    ROS_INFO_STREAM("publishing pointcloud on topic : " << this->topic_name << " in frame " << this->frame_name );


    // Advertise publisher
    ros::AdvertiseOptions ao = ros::AdvertiseOptions::create<sensor_msgs::PointCloud2>(
        this->topic_name, 5,
        boost::bind(&LivoxPointsPlugin::ConnectCb, this),
        boost::bind(&LivoxPointsPlugin::ConnectCb, this),
        ros::VoidPtr(), &this->laser_queue_
    );
    this->ros_pub = this->ros_node->advertise(ao);

    // Start custom queue for laser
    this->callback_laser_queue_thread_ = boost::thread( boost::bind( &LivoxPointsPlugin::laserQueueThread,this ) );

    // Sensor generation off by default
    this->parent_sensor->SetActive(false);

    // prepare the rays in their initial configuration
    this->UpdateRays();
    
    // check parent sensor 
    ROS_INFO_STREAM("RANGE    : " << ray_shape->GetMinRange() << " - " << ray_shape->GetMaxRange() << " m");
    ROS_INFO_STREAM("HZ ANGLE : " << parent_sensor->AngleMin().Degree() << " - " << parent_sensor->AngleMax().Degree() << " deg");
    ROS_INFO_STREAM("VT ANGLE : " << parent_sensor->VerticalAngleMin().Degree() << " - " << parent_sensor->VerticalAngleMax().Degree() << " deg");
    ROS_INFO_STREAM("RESLTION : " << parent_sensor->RayCount() << " x " << parent_sensor->VerticalRayCount() << " points");
    ROS_INFO_STREAM("Active   : " << (parent_sensor->IsActive() ? "YES" : "NO"));

}

void LivoxPointsPlugin::UpdateRays() {

    ignition::math::Vector3d start_point, 
                             end_point,
                             axis, 
                             unit_x(1.0, 0.0, 0.0);
    ignition::math::Quaterniond ray;

    auto offset = this->parent_sensor->Pose();

    // update ray positions
    this->parent_sensor->SetActive(false); // lock the sensor while we're modifying it
    for (int i = 0; i < this->samples_step; i++) {
        int rotate_idx   = (this->current_samples_idx + i) % this->ray_rotations.size();
        auto &rotate_info = this->ray_rotations[rotate_idx];

        // calculated the ray axis
        ray.Euler(0.0, rotate_info.zenith, rotate_info.azimuth);
        axis = offset.Rot() * ray * unit_x;

        // start & end points of the ray
        start_point = this->ray_shape->GetMinRange() * axis + offset.Pos();
        end_point = this->ray_shape->GetMaxRange() * axis + offset.Pos();

        // create new rays / update ray info
        if (i < this->ray_shape->RayCount()) 
            this->ray_shape->SetRay(i, start_point, end_point);
        else 
            this->ray_shape->AddRay(start_point, end_point);
    }
    this->parent_sensor->SetActive(true); // unlock the sensor 

    // update start index for next iteration
    this->current_samples_idx += this->samples_step;

}

// Subscribe on-demand
void LivoxPointsPlugin::ConnectCb()
{
  if (this->ros_pub.getNumSubscribers()) {
    if (!this->gz_sub) {
      this->gz_sub = this->gz_node->Subscribe(this->parent_sensor->Topic(), &LivoxPointsPlugin::OnScan, this);
    }
    this->parent_sensor->SetActive(true);
  } else {
    if (this->gz_sub) {
      this->gz_sub->Unsubscribe();
      this->gz_sub.reset();
    }
    this->parent_sensor->SetActive(false);
  }
}


// translate gazebo message to ROS pointcloud message
void LivoxPointsPlugin::OnScan(ConstLaserScanStampedPtr& _msg) {

    // Populate message fields
    sensor_msgs::PointCloud2 msg;
    msg.fields.resize(6);
    msg.fields[POINTFIELD_X].name = "x";
    msg.fields[POINTFIELD_X].offset = 0;
    msg.fields[POINTFIELD_X].datatype = sensor_msgs::PointField::FLOAT32;
    msg.fields[POINTFIELD_X].count = 1;
    msg.fields[POINTFIELD_Y].name = "y";
    msg.fields[POINTFIELD_Y].offset = msg.fields[POINTFIELD_X].offset + 4;
    msg.fields[POINTFIELD_Y].datatype = sensor_msgs::PointField::FLOAT32;
    msg.fields[POINTFIELD_Y].count = 1;
    msg.fields[POINTFIELD_Z].name = "z";
    msg.fields[POINTFIELD_Z].offset = msg.fields[POINTFIELD_Y].offset + 4;
    msg.fields[POINTFIELD_Z].datatype = sensor_msgs::PointField::FLOAT32;
    msg.fields[POINTFIELD_Z].count = 1;
    msg.fields[POINTFIELD_DST].name = "distance";
    msg.fields[POINTFIELD_DST].offset = msg.fields[POINTFIELD_Z].offset + 4;
    msg.fields[POINTFIELD_DST].datatype = sensor_msgs::PointField::FLOAT32;
    msg.fields[POINTFIELD_DST].count = 1;
    msg.fields[POINTFIELD_INT].name = "intensity";
    msg.fields[POINTFIELD_INT].offset = msg.fields[POINTFIELD_DST].offset + 4;
    msg.fields[POINTFIELD_INT].datatype = sensor_msgs::PointField::FLOAT32;
    msg.fields[POINTFIELD_INT].count = 1;
    msg.fields[POINTFIELD_CHN].name = "tag";
    msg.fields[POINTFIELD_CHN].offset = msg.fields[POINTFIELD_INT].offset + 4;
    msg.fields[POINTFIELD_CHN].datatype = sensor_msgs::PointField::UINT8;
    msg.fields[POINTFIELD_CHN].count = 1;

    // message header
    msg.header.frame_id = this->frame_name;
    msg.header.stamp = ros::Time(_msg->time().sec(), _msg->time().nsec());

    // metadata 
    msg.width = this->ray_shape->RayCount();
    msg.height = 1;
    msg.point_step = msg.fields[POINTFIELD_CHN].offset + 1; 
    msg.row_step =  msg.width * msg.point_step; 
    msg.data.resize(msg.height * msg.row_step);


    struct Point_t {
        float x, y, z, range, intensity;
    } point;
    float nan = std::nan("0");
    float inv_sqrt_3 = 1/sqrt(3.0);

    // fill in data
    auto* data = msg.data.data();
    auto offset = this->parent_sensor->Pose().Pos();
    this->parent_sensor->SetActive(false); // lock the sensor while we're reading it
    for (int row = 0; row < msg.height; row++) {
        for (int col = 0; col < msg.width; col++) {
            int ray_index = row*msg.width + col;
            int data_index = msg.point_step * ray_index;

            double true_range = this->ray_shape->GetRange(ray_index); // clean range
            double noise_range = this->parent_sensor->Range(ray_index); // noisy range
            if (true_range > this->ray_shape->GetMinRange() && true_range < this->ray_shape->GetMaxRange()){
                auto noise = noise_range/true_range;
                // reconstruct noisy point, assuming isotropy of noise
                auto p = inv_sqrt_3*noise*(this->ray_shape->Ray(ray_index)->End() - offset);
                point.x = p.X();
                point.y = p.Y();
                point.z = p.Z();
                point.range = noise_range;
            } else {
                point.x = nan;
                point.y = nan;
                point.z = nan;
                point.range = nan;
            }

            point.intensity = this->parent_sensor->Retro(ray_index); 

            // we copy the structured point data into the vector
            memcpy((void*) (data + data_index), &point, sizeof(Point_t)); 
            data[data_index + POINTFIELD_CHN] = (uint8_t) 0; // TODO : use a more realistic channel
        }
    }
    this->parent_sensor->SetActive(true); // unlock the sensor 

    // publish ros message
    if (this->ros_pub) 
        this->ros_pub.publish(msg);

    // prepare the rays in preparation for the next scan
    this->UpdateRays();
}


// Custom Callback Queue
////////////////////////////////////////////////////////////////////////////////
// Custom callback queue thread
void LivoxPointsPlugin::laserQueueThread()
{
  while (this->ros_node->ok()) {
    this->laser_queue_.callAvailable(ros::WallDuration(0.01));
  }
}