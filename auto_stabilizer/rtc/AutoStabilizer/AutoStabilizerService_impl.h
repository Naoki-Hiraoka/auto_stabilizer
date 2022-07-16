// -*-C++-*-
#ifndef AutoStabilizerSERVICESVC_IMPL_H
#define AutoStabilizerSERVICESVC_IMPL_H

#include "whole_body_position_controller/idl/AutoStabilizerService.hh"

class AutoStabilizer;

class AutoStabilizerService_impl
  : public virtual POA_whole_body_position_controller::AutoStabilizerService,
    public virtual PortableServer::RefCountServantBase
{
public:
  AutoStabilizerService_impl();// 実装は.cppファイルの方に書かないと、registerProvider時にSegmentation Faultになる
  ~AutoStabilizerService_impl();
  CORBA::Boolean startControl();
  CORBA::Boolean stopControl();
  void setParams(const whole_body_position_controller::AutoStabilizerService::AutoStabilizerParam& i_param);
  void getParams(whole_body_position_controller::AutoStabilizerService::AutoStabilizerParam_out i_param);
  //
  void setComp(AutoStabilizer *i_comp);
private:
  AutoStabilizer *comp_;
};

#endif
