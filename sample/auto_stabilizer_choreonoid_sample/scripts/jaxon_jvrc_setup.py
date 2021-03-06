#!/usr/bin/env python

from hrpsys_choreonoid_tutorials.choreonoid_hrpsys_config import *

import OpenRTM_aist
import OpenHRP
from hrpsys.RobotHardwareService_idl import *
from auto_stabilizer.AutoStabilizerService_idl import *

from auto_stabilizer import *

class JAXON_RED_HrpsysConfigurator(ChoreonoidHrpsysConfigurator):
    ast = None
    ast_svc = None
    ast_version = None

    def getRTCList (self):
        ##return self.getRTCListUnstable()
        return [
            ['seq', "SequencePlayer"],
            ['sh', "StateHolder"],
            ['fk', "ForwardKinematics"],
            ['tf', "TorqueFilter"],
            ['kf', "KalmanFilter"],
            ['vs', "VirtualForceSensor"],
            ['rmfo', "RemoveForceSensorLinkOffset"],
            ['es', "EmergencyStopper"],
            ['ast', "AutoStabilizer"],
            ['co', "CollisionDetector"],
            ['hes', "EmergencyStopper"],
            ['el', "SoftErrorLimiter"],
            ['log', "DataLogger"]
            ]

    def getJointAngleControllerList(self):
        controller_list = [self.es, self.ic, self.gc, self.abc, self.st, self.ast, self.co,
                           self.tc, self.hes, self.el] # ast is added
        return filter(lambda c: c != None, controller_list)  # only return existing controllers

    def connectComps(self):
        super(JAXON_RED_HrpsysConfigurator, self).connectComps()
        if self.ast:
            connectPorts(self.sh.port("basePosOut"), self.ast.port("refBasePosIn"))
            connectPorts(self.sh.port("baseRpyOut"), self.ast.port("refBaseRpyIn"))
            connectPorts(self.rh.port("q"), self.ast.port("qAct"))
            connectPorts(self.rh.port("dq"), self.ast.port("dqAct"))
            if self.st:
                disconnectPorts(self.st.port("tau"), self.rh.port("tauRef"))
                connectPorts(self.st.port("tau"), self.ast.port("refTauIn"))
                connectPorts(self.ast.port("genTauOut"), self.rh.port("tauRef"))
            else:
                disconnectPorts(self.sh.port("tqOut"), self.rh.port("tauRef"))
                connectPorts(self.sh.port("tqOut"), self.ast.port("refTauIn"))
                connectPorts(self.ast.port("genTauOut"), self.rh.port("tauRef"))
            if self.kf:
                connectPorts(self.kf.port("rpy"), self.ast.port("actImuIn"))
            for sen, eef in zip(["rfsensor", "lfsensor", "rhsensor", "lhsensor"], ["rleg", "lleg", "rarm", "larm"]):
                if self.rfu:
                    ref_force_port_from = self.rfu.port("ref_"+sen+"Out")
                elif self.es:
                    ref_force_port_from = self.es.port(sen+"Out")
                else:
                    ref_force_port_from = self.sh.port(sen+"Out")
                connectPorts(ref_force_port_from, self.ast.port("ref" + eef + "WrenchIn"))
            for sen in self.getForceSensorNames():
                if self.rmfo:
                    connectPorts(self.rmfo.port("off_" + sen), self.ast.port("act"+sen+"In"))
                else:
                    connectPorts(self.rh.port(sen), self.ast.port("act"+sen+"In"))

    def defJointGroups (self):
        rarm_group = ['rarm', ['RARM_JOINT0', 'RARM_JOINT1', 'RARM_JOINT2', 'RARM_JOINT3', 'RARM_JOINT4', 'RARM_JOINT5', 'RARM_JOINT6', 'RARM_JOINT7']]
        larm_group = ['larm', ['LARM_JOINT0', 'LARM_JOINT1', 'LARM_JOINT2', 'LARM_JOINT3', 'LARM_JOINT4', 'LARM_JOINT5', 'LARM_JOINT6', 'LARM_JOINT7']]
        rleg_group = ['rleg', ['RLEG_JOINT0', 'RLEG_JOINT1', 'RLEG_JOINT2', 'RLEG_JOINT3', 'RLEG_JOINT4', 'RLEG_JOINT5']]
        lleg_group = ['lleg', ['LLEG_JOINT0', 'LLEG_JOINT1', 'LLEG_JOINT2', 'LLEG_JOINT3', 'LLEG_JOINT4', 'LLEG_JOINT5']]
        head_group = ['head', ['HEAD_JOINT0', 'HEAD_JOINT1']]
        torso_group = ['torso', ['CHEST_JOINT0', 'CHEST_JOINT1', 'CHEST_JOINT2']]
        rhand_group = ['rhand', ['RARM_F_JOINT0', 'RARM_F_JOINT1']]
        lhand_group = ['lhand', ['LARM_F_JOINT0', 'LARM_F_JOINT1']]
        self.Groups = [rarm_group, larm_group, rleg_group, lleg_group, head_group, torso_group, rhand_group, lhand_group]

    def startABSTIMP (self):
        ### not used on hrpsys
        self.el_svc.setServoErrorLimit("RARM_F_JOINT0", sys.float_info.max)
        self.el_svc.setServoErrorLimit("RARM_F_JOINT1", sys.float_info.max)
        self.el_svc.setServoErrorLimit("LARM_F_JOINT0", sys.float_info.max)
        self.el_svc.setServoErrorLimit("LARM_F_JOINT1", sys.float_info.max)
        ###
        #self.startAutoBalancer()
        # Suppress limit over message and behave like real robot that always angle-vector is in seq.
        # Latter four 0.0 are for hands.
        self.seq_svc.setJointAngles(self.jaxonResetPose()+[0.0, 0.0, 0.0, 0.0] , 1.0)
        #self.ic_svc.startImpedanceController("larm")
        #self.ic_svc.startImpedanceController("rarm")
        #self.startStabilizer()

if __name__ == '__main__':
    hcf = JAXON_RED_HrpsysConfigurator("JAXON_RED")
    [sys.argv, connect_constraint_force_logger_ports] = hcf.parse_arg_for_connect_ports(sys.argv)
    if len(sys.argv) > 2 :
        hcf.init(sys.argv[1], sys.argv[2], connect_constraint_force_logger_ports=connect_constraint_force_logger_ports)
        hcf.startABSTIMP()
    elif len(sys.argv) > 1 :
        hcf.init(sys.argv[1], connect_constraint_force_logger_ports=connect_constraint_force_logger_ports)
        hcf.startABSTIMP()
    else :
        hcf.init(connect_constraint_force_logger_ports=connect_constraint_force_logger_ports)
