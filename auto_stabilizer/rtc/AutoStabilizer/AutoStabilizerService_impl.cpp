#include "AutoStabilizerService_impl.h"
#include "AutoStabilizer.h"

AutoStabilizerService_impl::AutoStabilizerService_impl()
{
}

AutoStabilizerService_impl::~AutoStabilizerService_impl()
{
}

void AutoStabilizerService_impl::setComp(AutoStabilizer *i_comp)
{
  comp_ = i_comp;
}

CORBA::Boolean AutoStabilizerService_impl::startControl()
{
  return comp_->startControl();
};

CORBA::Boolean AutoStabilizerService_impl::stopControl()
{
  return comp_->stopControl();
};

void AutoStabilizerService_impl::setParams(const whole_body_position_controller::AutoStabilizerService::AutoStabilizerParam& i_param)
{
  comp_->setParams(i_param);
};

void AutoStabilizerService_impl::getParams(whole_body_position_controller::AutoStabilizerService::AutoStabilizerParam_out i_param)
{
  i_param = new whole_body_position_controller::AutoStabilizerService::AutoStabilizerParam();
  comp_->getParams(*i_param);
};

