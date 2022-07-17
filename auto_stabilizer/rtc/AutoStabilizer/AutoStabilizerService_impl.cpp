#include "AutoStabilizerService_impl.h"
#include "AutoStabilizer.h"

AutoStabilizerService_impl::AutoStabilizerService_impl()
{
}

AutoStabilizerService_impl::~AutoStabilizerService_impl()
{
}

CORBA::Boolean AutoStabilizerService_impl::goPos( CORBA::Double x,  CORBA::Double y,  CORBA::Double th)
{
  return this->comp_->goPos(x, y, th);
};

CORBA::Boolean AutoStabilizerService_impl::goVelocity( CORBA::Double vx,  CORBA::Double vy,  CORBA::Double vth)
{
  return this->comp_->goVelocity(vx, vy, vth);
};

CORBA::Boolean AutoStabilizerService_impl::goStop()
{
  return this->comp_->goStop();
};

CORBA::Boolean AutoStabilizerService_impl::jumpTo( CORBA::Double x,  CORBA::Double y,  CORBA::Double z,  CORBA::Double ts,  CORBA::Double tf)
{
  return this->comp_->jumpTo(x, y, z, ts, tf);
};

CORBA::Boolean AutoStabilizerService_impl::emergencyStop()
{
  return this->comp_->emergencyStop();
};

CORBA::Boolean AutoStabilizerService_impl::setFootSteps(const OpenHRP::AutoStabilizerService::FootstepsSequence& fss, CORBA::Long overwrite_fs_idx)
{
  return this->comp_->setFootSteps(fss, overwrite_fs_idx);
}

CORBA::Boolean AutoStabilizerService_impl::setFootStepsWithParam(const OpenHRP::AutoStabilizerService::FootstepsSequence& fss, const OpenHRP::AutoStabilizerService::StepParamsSequence& spss, CORBA::Long overwrite_fs_idx)
{
  return this->comp_->setFootStepsWithParam(fss, spss, overwrite_fs_idx);
}

void AutoStabilizerService_impl::waitFootSteps()
{
  return this->comp_->waitFootSteps();
};

CORBA::Boolean AutoStabilizerService_impl::startAutoBalancer(const OpenHRP::AutoStabilizerService::StrSequence& limbs)
{
  return this->comp_->startAutoBalancer(limbs);
};

CORBA::Boolean AutoStabilizerService_impl::stopAutoBalancer()
{
  return this->comp_->stopAutoBalancer();
};

CORBA::Boolean AutoStabilizerService_impl::startStabilizer(void)
{
  return this->comp_->startStabilizer();
}

CORBA::Boolean AutoStabilizerService_impl::stopStabilizer(void)
{
  return this->comp_->stopStabilizer();
}

CORBA::Boolean AutoStabilizerService_impl::setGaitGeneratorParam(const OpenHRP::AutoStabilizerService::GaitGeneratorParam& i_param)
{
  return this->comp_->setGaitGeneratorParam(i_param);
};

CORBA::Boolean AutoStabilizerService_impl::getGaitGeneratorParam(OpenHRP::AutoStabilizerService::GaitGeneratorParam_out i_param)
{
  i_param = new OpenHRP::AutoStabilizerService::GaitGeneratorParam();
  i_param->stride_parameter.length(6);
  i_param->toe_heel_phase_ratio.length(7);
  i_param->zmp_weight_map.length(4);
  return this->comp_->getGaitGeneratorParam(*i_param);
};

CORBA::Boolean AutoStabilizerService_impl::setAutoBalancerParam(const OpenHRP::AutoStabilizerService::AutoBalancerParam& i_param)
{
  return this->comp_->setAutoBalancerParam(i_param);
};

CORBA::Boolean AutoStabilizerService_impl::getAutoBalancerParam(OpenHRP::AutoStabilizerService::AutoBalancerParam_out i_param)
{
  i_param = new OpenHRP::AutoStabilizerService::AutoBalancerParam();
  return this->comp_->getAutoBalancerParam(*i_param);
};

void AutoStabilizerService_impl::setStabilizerParam(const OpenHRP::AutoStabilizerService::StabilizerParam& i_param)
{
  this->comp_->setStabilizerParam(i_param);
}

void AutoStabilizerService_impl::getStabilizerParam(OpenHRP::AutoStabilizerService::StabilizerParam_out i_param)
{
  i_param = new OpenHRP::AutoStabilizerService::StabilizerParam();
  return this->comp_->getStabilizerParam(*i_param);
}

CORBA::Boolean AutoStabilizerService_impl::releaseEmergencyStop()
{
    return this->comp_->releaseEmergencyStop();
};

void AutoStabilizerService_impl::setComp(AutoStabilizer *i_comp)
{
  this->comp_ = i_comp;
}
