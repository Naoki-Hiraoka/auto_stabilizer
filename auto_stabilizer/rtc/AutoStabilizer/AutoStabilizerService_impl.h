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
  CORBA::Boolean setFootSteps(const OpenHRP::AutoStabilizerService::FootstepSequence& fs);
  CORBA::Boolean setFootStepsWithParam(const OpenHRP::AutoStabilizerService::FootstepSequence& fs, const OpenHRP::AutoStabilizerService::StepParamSequence& spss);
  void waitFootSteps();
  CORBA::Boolean startAutoBalancer();
  CORBA::Boolean stopAutoBalancer();
  CORBA::Boolean startStabilizer(void);
  CORBA::Boolean stopStabilizer(void);
  CORBA::Boolean setAutoStabilizerParam(const OpenHRP::AutoStabilizerService::AutoStabilizerParam& i_param);
  CORBA::Boolean getAutoStabilizerParam(OpenHRP::AutoStabilizerService::AutoStabilizerParam_out i_param);
  CORBA::Boolean getFootStepState(OpenHRP::AutoStabilizerService::FootStepState_out i_param);
  CORBA::Boolean releaseEmergencyStop();
  CORBA::Boolean startImpedanceController(const char *i_name_);
  CORBA::Boolean stopImpedanceController(const char *i_name_);
  CORBA::Boolean startWholeBodyMasterSlave();
  CORBA::Boolean stopWholeBodyMasterSlave();
  //
  //
  void setComp(AutoStabilizer *i_comp);
private:
  AutoStabilizer *comp_;
};

#endif
