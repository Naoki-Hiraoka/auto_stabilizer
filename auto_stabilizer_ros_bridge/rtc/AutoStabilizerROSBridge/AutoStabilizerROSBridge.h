#ifndef AutoStabilizerROSBridge_H
#define AUtoStabilizerROSBridge_H

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/DataOutPort.h>
#include <rtm/DataInPort.h>

#include <auto_stabilizer_msgs/idl/AutoStabilizer.hh>
#include <auto_stabilizer_msgs/SteppableRegion.h>
#include <auto_stabilizer_msgs/LandingPosition.h>

#include <std_msgs/Float32.h>

#include <ros/ros.h>

class AutoStabilizerROSBridge : public RTC::DataFlowComponentBase{
protected:
  ros::NodeHandle nh;

  auto_stabilizer_msgs::TimedLandingPosition m_landingTarget_;
  RTC::InPort <auto_stabilizer_msgs::TimedLandingPosition> m_landingTargetIn_;
  ros::Publisher landing_target_pub_;

  RTC::TimedDoubleSeq m_legOdom_;
  RTC::InPort <RTC::TimedDoubleSeq> m_legOdomIn_;

  RTC::TimedPoint3D m_actZmp_;
  RTC::InPort <RTC::TimedPoint3D> m_actZmpIn_;
  ros::Publisher actZmp_pub_;

  RTC::TimedPoint3D m_tgtZmp_;
  RTC::InPort <RTC::TimedPoint3D> m_tgtZmpIn_;
  ros::Publisher tgtZmp_pub_;

  RTC::TimedPoint3D m_actCog_;
  RTC::InPort <RTC::TimedPoint3D> m_actCogIn_;
  ros::Publisher actCog_pub_;

  ros::Subscriber steppable_region_sub_;
  auto_stabilizer_msgs::TimedSteppableRegion m_steppableRegion_;
  RTC::OutPort <auto_stabilizer_msgs::TimedSteppableRegion> m_steppableRegionOut_;

  ros::Subscriber landing_height_sub_;
  auto_stabilizer_msgs::TimedLandingPosition m_landingHeight_;
  RTC::OutPort <auto_stabilizer_msgs::TimedLandingPosition> m_landingHeightOut_;

  ros::Subscriber wheel_vel_sub_;
  RTC::TimedFloat m_wheelVel_;
  RTC::OutPort <RTC::TimedFloat> m_wheelVelOut_;
public:
  AutoStabilizerROSBridge(RTC::Manager* manager);
  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

  void onSteppableRegionCB(const auto_stabilizer_msgs::SteppableRegion::ConstPtr& msg);
  void onLandingHeightCB(const auto_stabilizer_msgs::LandingPosition::ConstPtr& msg);
  void onWheelVelCB(const std_msgs::Float32::ConstPtr& msg);

};

extern "C"
{
  void AutoStabilizerROSBridgeInit(RTC::Manager* manager);
};

#endif // AutoStabilizerROSBridge_H
