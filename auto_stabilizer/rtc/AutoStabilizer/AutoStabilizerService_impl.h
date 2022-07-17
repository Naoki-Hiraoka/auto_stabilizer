// -*-C++-*-
#ifndef AutoStabilizerSERVICESVC_IMPL_H
#define AutoStabilizerSERVICESVC_IMPL_H

#include "auto_stabilizer/idl/AutoStabilizerService.hh"

class AutoStabilizer;

class AutoStabilizerService_impl
  : public virtual POA_OpenHRP::AutoStabilizerService,
    public virtual PortableServer::RefCountServantBase
{
public:
  AutoStabilizerService_impl();// 実装は.cppファイルの方に書かないと、registerProvider時にSegmentation Faultになる
  ~AutoStabilizerService_impl();
  CORBA::Boolean goPos( CORBA::Double x,  CORBA::Double y,  CORBA::Double th);
  CORBA::Boolean goVelocity( CORBA::Double vx,  CORBA::Double vy,  CORBA::Double vth);
  CORBA::Boolean goStop();
  CORBA::Boolean jumpTo( CORBA::Double x,  CORBA::Double y,  CORBA::Double z,  CORBA::Double ts,  CORBA::Double tf);
  CORBA::Boolean emergencyStop();
  CORBA::Boolean setFootSteps(const OpenHRP::AutoStabilizerService::FootstepsSequence& fss, CORBA::Long overwrite_fs_idx);
  CORBA::Boolean setFootStepsWithParam(const OpenHRP::AutoStabilizerService::FootstepsSequence& fss, const OpenHRP::AutoStabilizerService::StepParamsSequence& spss, CORBA::Long overwrite_fs_idx);
  void waitFootSteps();
  CORBA::Boolean startAutoBalancer(const OpenHRP::AutoStabilizerService::StrSequence& limbs);
  CORBA::Boolean stopAutoBalancer();
  void startStabilizer(void);
  void stopStabilizer(void);
  CORBA::Boolean setGaitGeneratorParam(const OpenHRP::AutoStabilizerService::GaitGeneratorParam& i_param);
  CORBA::Boolean getGaitGeneratorParam(OpenHRP::AutoStabilizerService::GaitGeneratorParam_out i_param);
  CORBA::Boolean setAutoBalancerParam(const OpenHRP::AutoStabilizerService::AutoBalancerParam& i_param);
  CORBA::Boolean getAutoBalancerParam(OpenHRP::AutoStabilizerService::AutoBalancerParam_out i_param);
  void setStabilizerParam(const OpenHRP::AutoStabilizerService::StabilizerParam& i_param);
  void getStabilizerParam(OpenHRP::AutoStabilizerService::StabilizerParam_out i_param);
  CORBA::Boolean releaseEmergencyStop();
  //
  //
  void setComp(AutoStabilizer *i_comp);
private:
  AutoStabilizer *comp_;
};

#endif
