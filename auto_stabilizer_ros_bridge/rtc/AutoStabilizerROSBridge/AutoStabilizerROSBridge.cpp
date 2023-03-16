#include "AutoStabilizerROSBridge.h"
#include <tf/transform_broadcaster.h>

AutoStabilizerROSBridge::AutoStabilizerROSBridge(RTC::Manager* manager):
  RTC::DataFlowComponentBase(manager),
  m_steppableRegionOut_("steppableRegionOut", m_steppableRegion_),
  m_landingHeightOut_("landingHeightOut", m_landingHeight_),
  m_wheelVelOut_("wheelVelOut", m_wheelVel_),
  m_landingTargetIn_("landingTargetIn", m_landingTarget_),
  m_legOdomIn_("legOdomIn", m_legOdom_),
  m_actZmpIn_("actZmpIn", m_actZmp_),
  m_tgtZmpIn_("tgtZmpIn", m_tgtZmp_),
  m_actCogIn_("actCogIn", m_actCog_)
{
}

RTC::ReturnCode_t AutoStabilizerROSBridge::onInitialize(){
  addOutPort("steppableRegionOut", m_steppableRegionOut_);
  addOutPort("landingHeightOut", m_landingHeightOut_);
  addOutPort("wheelVelOut", m_wheelVelOut_);
  addInPort("landingTargetIn", m_landingTargetIn_);
  addInPort("legOdomIn", m_legOdomIn_);
  addInPort("actZmpIn", m_actZmpIn_);
  addInPort("tgtZmpIn", m_tgtZmpIn_);
  addInPort("actCogIn", m_actCogIn_);

  ros::NodeHandle pnh("~");

  steppable_region_sub_ = pnh.subscribe("steppable_region", 1, &AutoStabilizerROSBridge::onSteppableRegionCB, this);
  landing_height_sub_ = pnh.subscribe("landing_height", 1, &AutoStabilizerROSBridge::onLandingHeightCB, this);
  wheel_vel_sub_ = pnh.subscribe("wheel_vel", 1, &AutoStabilizerROSBridge::onWheelVelCB, this);
  landing_target_pub_ = pnh.advertise<auto_stabilizer_msgs::LandingPosition>("landing_target", 1);
  actZmp_pub_ = pnh.advertise<geometry_msgs::PointStamped>("actZmp", 1);
  tgtZmp_pub_ = pnh.advertise<geometry_msgs::PointStamped>("tgtZmp", 1);
  actCog_pub_ = pnh.advertise<geometry_msgs::PointStamped>("actCog", 1);

  return RTC::RTC_OK;
}

RTC::ReturnCode_t AutoStabilizerROSBridge::onExecute(RTC::UniqueId ec_id){
  ros::spinOnce();
  if(this->m_landingTargetIn_.isNew()){
    try {
      m_landingTargetIn_.read();
      auto_stabilizer_msgs::LandingPosition landingTarget;
      landingTarget.header.stamp = ros::Time::now();
      // landingTarget.header.frame_id -- no used
      landingTarget.x = m_landingTarget_.data.x;
      landingTarget.y = m_landingTarget_.data.y;
      landingTarget.z = m_landingTarget_.data.z;
      landingTarget.l_r = m_landingTarget_.data.l_r;
      landing_target_pub_.publish(landingTarget);
    }
    catch(const std::runtime_error &e)
      {
        ROS_ERROR_STREAM("[" << getInstanceName() << "] " << e.what());
      }
  }

  if(this->m_legOdomIn_.isNew()){
    try {
      m_legOdomIn_.read();
      static tf::TransformBroadcaster br;
      tf::Transform transform;
      transform.setOrigin(tf::Vector3(m_legOdom_.data[0], m_legOdom_.data[1], m_legOdom_.data[2]));
      tf::Quaternion q(m_legOdom_.data[3], m_legOdom_.data[4], m_legOdom_.data[5], m_legOdom_.data[6]);
      transform.setRotation(q);
      if (m_legOdom_.data[7] == 0) {
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "rleg_end_coords", "leg_odom"));
      } else {
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "lleg_end_coords", "leg_odom"));
      }
    }
    catch(const std::runtime_error &e)
      {
        ROS_ERROR_STREAM("[" << getInstanceName() << "] " << e.what());
      }
  }

  if(this->m_actZmpIn_.isNew()){
    try {
      m_actZmpIn_.read();
      geometry_msgs::PointStamped actZmpMsg;
      actZmpMsg.header.stamp = ros::Time::now();
      actZmpMsg.point.x = m_actZmp_.data.x;
      actZmpMsg.point.y = m_actZmp_.data.y;
      actZmpMsg.point.z = m_actZmp_.data.z;
      actZmp_pub_.publish(actZmpMsg);
    }
    catch(const std::runtime_error &e)
      {
        ROS_ERROR_STREAM("[" << getInstanceName() << "] " << e.what());
      }
  }

  if(this->m_tgtZmpIn_.isNew()){
    try {
      m_tgtZmpIn_.read();
      geometry_msgs::PointStamped tgtZmpMsg;
      tgtZmpMsg.header.stamp = ros::Time::now();
      tgtZmpMsg.point.x = m_tgtZmp_.data.x;
      tgtZmpMsg.point.y = m_tgtZmp_.data.y;
      tgtZmpMsg.point.z = m_tgtZmp_.data.z;
      tgtZmp_pub_.publish(tgtZmpMsg);
    }
    catch(const std::runtime_error &e)
      {
        ROS_ERROR_STREAM("[" << getInstanceName() << "] " << e.what());
      }
  }

  if(this->m_actCogIn_.isNew()){
    try {
      m_actCogIn_.read();
      geometry_msgs::PointStamped actCogMsg;
      actCogMsg.header.stamp = ros::Time::now();
      actCogMsg.point.x = m_actCog_.data.x;
      actCogMsg.point.y = m_actCog_.data.y;
      actCogMsg.point.z = m_actCog_.data.z;
      actCog_pub_.publish(actCogMsg);
    }
    catch(const std::runtime_error &e)
      {
        ROS_ERROR_STREAM("[" << getInstanceName() << "] " << e.what());
      }
  }
  return RTC::RTC_OK;
}

void AutoStabilizerROSBridge::onSteppableRegionCB(const auto_stabilizer_msgs::SteppableRegion::ConstPtr& msg) {
  size_t convex_num(msg->polygons.size());
  m_steppableRegion_.data.region.length(convex_num);
  for (size_t i = 0; i < convex_num; i++) {
    size_t vs_num(msg->polygons[i].polygon.points.size());
    m_steppableRegion_.data.region[i].length(3 * vs_num); // x,y,z components
    for (size_t j = 0; j < vs_num; j++) {
      m_steppableRegion_.data.region[i][3*j] = msg->polygons[i].polygon.points[j].x;
      m_steppableRegion_.data.region[i][3*j+1] = msg->polygons[i].polygon.points[j].y;
      m_steppableRegion_.data.region[i][3*j+2] = msg->polygons[i].polygon.points[j].z;
    }
  }
  m_steppableRegion_.data.l_r = msg->l_r;
  m_steppableRegionOut_.write();
}

void AutoStabilizerROSBridge::onLandingHeightCB(const auto_stabilizer_msgs::LandingPosition::ConstPtr& msg) {
  m_landingHeight_.data.x = msg->x;
  m_landingHeight_.data.y = msg->y;
  m_landingHeight_.data.z = msg->z;
  m_landingHeight_.data.nx = msg->nx;
  m_landingHeight_.data.ny = msg->ny;
  m_landingHeight_.data.nz = msg->nz;
  m_landingHeight_.data.l_r = msg->l_r;
  m_landingHeightOut_.write();
}

void AutoStabilizerROSBridge::onWheelVelCB(const std_msgs::Float32::ConstPtr& msg) {
  m_wheelVel_.data = msg->data;
  m_wheelVelOut_.write();
  std::cout << "ros_bridge wheel vel: " << msg->data << std::endl;
}

static const char* AutoStabilizerROSBridge_spec[] = {
  "implementation_id", "AutoStabilizerROSBridge",
  "type_name",         "AutoStabilizerROSBridge",
  "description",       "AutoStabilizerROSBridge component",
  "version",           "0.0",
  "vendor",            "Takuma-Hiraoka",
  "category",          "example",
  "activity_type",     "DataFlowComponent",
  "max_instance",      "10",
  "language",          "C++",
  "lang_type",         "compile",
  ""
};

extern "C"{
    void AutoStabilizerROSBridgeInit(RTC::Manager* manager) {
        RTC::Properties profile(AutoStabilizerROSBridge_spec);
        manager->registerFactory(profile, RTC::Create<AutoStabilizerROSBridge>, RTC::Delete<AutoStabilizerROSBridge>);
    }
};
