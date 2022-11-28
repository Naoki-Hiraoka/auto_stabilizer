#include "AutoStabilizerROSBridge.h"

AutoStabilizerROSBridge::AutoStabilizerROSBridge(RTC::Manager* manager):
  RTC::DataFlowComponentBase(manager),
  m_steppableRegionOut_("steppableRegionOut", m_steppableRegion_),
  m_landingHeightOut_("landingHeightOut", m_landingHeight_),
  m_landingTargetIn_("landingTargetIn", m_landingTarget_)
{
}

RTC::ReturnCode_t AutoStabilizerROSBridge::onInitialize(){
  addOutPort("steppableRegionOut", m_steppableRegionOut_);
  addOutPort("landingHeightOut", m_landingHeightOut_);
  addInPort("landingTargetIn", m_landingTargetIn_);

  ros::NodeHandle pnh("~");

  steppable_region_sub_ = pnh.subscribe("steppable_region", 1, &AutoStabilizerROSBridge::onSteppableRegionCB, this);
  landing_height_sub_ = pnh.subscribe("landing_height", 1, &AutoStabilizerROSBridge::onLandingHeightCB, this);
  landing_target_pub_ = pnh.advertise<auto_stabilizer_msgs::LandingPosition>("landing_target", 1);

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
