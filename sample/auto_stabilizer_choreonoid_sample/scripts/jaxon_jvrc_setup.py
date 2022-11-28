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
            if self.kf:
                connectPorts(self.ast.port("genImuAccOut"), self.kf.port("accRef"))
            connectPorts(self.ast.port("RobotHardwareService"), self.rh.port("RobotHardwareService"))

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

    def setupLogger(self):
        if self.ast:
            self.log_svc.add("TimedDoubleSeq","ast_q")
            rtm.connectPorts(rtm.findRTC("ast").port("q"),rtm.findRTC("log").port("ast_q"))
            self.log_svc.add("TimedPoint3D","ast_genCogOut")
            rtm.connectPorts(rtm.findRTC("ast").port("genCogOut"),rtm.findRTC("log").port("ast_genCogOut"))
            self.log_svc.add("TimedPoint3D","ast_genDcmOut")
            rtm.connectPorts(rtm.findRTC("ast").port("genDcmOut"),rtm.findRTC("log").port("ast_genDcmOut"))
            self.log_svc.add("TimedPoint3D","ast_genZmpOut")
            rtm.connectPorts(rtm.findRTC("ast").port("genZmpOut"),rtm.findRTC("log").port("ast_genZmpOut"))
            self.log_svc.add("TimedPoint3D","ast_tgtZmpOut")
            rtm.connectPorts(rtm.findRTC("ast").port("tgtZmpOut"),rtm.findRTC("log").port("ast_tgtZmpOut"))
            self.log_svc.add("TimedPoint3D","ast_actCogOut")
            rtm.connectPorts(rtm.findRTC("ast").port("actCogOut"),rtm.findRTC("log").port("ast_actCogOut"))
            self.log_svc.add("TimedPoint3D","ast_actDcmOut")
            rtm.connectPorts(rtm.findRTC("ast").port("actDcmOut"),rtm.findRTC("log").port("ast_actDcmOut"))
            self.log_svc.add("TimedDoubleSeq","ast_dstLandingPosOut")
            rtm.connectPorts(rtm.findRTC("ast").port("dstLandingPosOut"),rtm.findRTC("log").port("ast_dstLandingPosOut"))
            self.log_svc.add("TimedDoubleSeq","ast_remainTimeOut")
            rtm.connectPorts(rtm.findRTC("ast").port("remainTimeOut"),rtm.findRTC("log").port("ast_remainTimeOut"))
            self.log_svc.add("TimedDoubleSeq","ast_genCoordsOut")
            rtm.connectPorts(rtm.findRTC("ast").port("genCoordsOut"),rtm.findRTC("log").port("ast_genCoordsOut"))
            self.log_svc.add("TimedDoubleSeq","ast_captureRegionOut")
            rtm.connectPorts(rtm.findRTC("ast").port("captureRegionOut"),rtm.findRTC("log").port("ast_captureRegionOut"))
            self.log_svc.add("TimedDoubleSeq","ast_steppableRegionLogOut")
            rtm.connectPorts(rtm.findRTC("ast").port("steppableRegionLogOut"),rtm.findRTC("log").port("ast_steppableRegionLogOut"))
            self.log_svc.add("TimedDoubleSeq","ast_steppableRegionNumLogOut")
            rtm.connectPorts(rtm.findRTC("ast").port("steppableRegionNumLogOut"),rtm.findRTC("log").port("ast_steppableRegionNumLogOut"))
            self.log_svc.add("TimedDoubleSeq","ast_strideLimitationHullOut")
            rtm.connectPorts(rtm.findRTC("ast").port("strideLimitationHullOut"),rtm.findRTC("log").port("ast_strideLimitationHullOut"))
            self.log_svc.add("TimedDoubleSeq","ast_cpViewerLogOut")
            rtm.connectPorts(rtm.findRTC("ast").port("cpViewerLogOut"),rtm.findRTC("log").port("ast_cpViewerLogOut"))
            for ee in ["rleg", "lleg", "rarm", "larm"]:
                self.log_svc.add("TimedDoubleSeq","ast_tgt" + ee + "WrenchOut")
                rtm.connectPorts(rtm.findRTC("ast").port("tgt" + ee + "WrenchOut"),rtm.findRTC("log").port("ast_tgt" + ee + "WrenchOut"))
                self.log_svc.add("TimedDoubleSeq","ast_act" + ee + "WrenchOut")
                rtm.connectPorts(rtm.findRTC("ast").port("act" + ee + "WrenchOut"),rtm.findRTC("log").port("ast_act" + ee + "WrenchOut"))
        super(JAXON_RED_HrpsysConfigurator, self).setupLogger()

    def startABSTIMP (self):
        ### for torque control
        self.el_svc.setServoErrorLimit("all", sys.float_info.max)
        self.rh_svc.setServoErrorLimit("all", sys.float_info.max)
        ### not used on hrpsys
        if self.el:
            self.el_svc.setServoErrorLimit("RARM_F_JOINT0", sys.float_info.max)
            self.el_svc.setServoErrorLimit("RARM_F_JOINT1", sys.float_info.max)
            self.el_svc.setServoErrorLimit("LARM_F_JOINT0", sys.float_info.max)
            self.el_svc.setServoErrorLimit("LARM_F_JOINT1", sys.float_info.max)
        ###
        self.rh_svc.setJointControlMode("all",OpenHRP.RobotHardwareService.TORQUE)
        self.rh_svc.setServoTorqueGainPercentage("all",100)
        # ast setting
        astp=self.ast_svc.getAutoStabilizerParam()[1]
        astp.controllable_joints = self.Groups[0][1] + self.Groups[1][1] + self.Groups[2][1] + self.Groups[3][1] + self.Groups[4][1] + self.Groups[5][1] # remove hand joints
        astp.dq_weight[12:15] = [1e2]*3 # reduce chest joint move
        self.ast_svc.setAutoStabilizerParam(astp)
        self.ast_svc.startAutoBalancer()
        # Suppress limit over message and behave like real robot that always angle-vector is in seq.
        # Latter four 0.0 are for hands.
        self.seq_svc.setJointAngles(self.jaxonResetPose()+[0.0, 0.0, 0.0, 0.0] , 1.0)
        #self.ast_svc.startImpedanceController("larm")
        #self.ast_svc.startImpedanceController("rarm")
        self.ast_svc.startStabilizer()

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
