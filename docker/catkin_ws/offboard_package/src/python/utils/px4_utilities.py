#!/usr/bin/env python
# ROS python API
import rospy
import numpy as np
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from geometry_msgs.msg import Point, PoseStamped

# Flight modes class
# Flight modes are activated using ROS services
class fcuModes:
    def __init__(self, uav =''):
        self.uav = uav

    def setTakeoff(self, alt):
        rospy.wait_for_service(self.uav+'/mavros/cmd/takeoff')
        try:
            takeoffService = rospy.ServiceProxy(self.uav+'/mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
            takeoffService(altitude = alt)
        except rospy.ServiceException as e:
            print("service set_mode call failed: {}. Takeoff could not be set.".format(e))

    def setArm(self):
        rospy.wait_for_service(self.uav+'/mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy(self.uav+'/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(True)
        except rospy.ServiceException as e:
            print("service set_mode call failed: {}. Arm could not be set.".format(e))

    def setDisarm(self):
        rospy.wait_for_service(self.uav+'/mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy(self.uav+'/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(False)
        except rospy.ServiceException as e:
            print("service set_mode call failed: {}. Disarm could not be set.".format(e))

    def setStabilizedMode(self):
        rospy.wait_for_service(self.uav+'/mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy(self.uav+'/mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='STABILIZED')
        except rospy.ServiceException as e:
            print("service set_mode call failed: {}. Stabilizer Mode could not be set.".format(e))

    def setOffboardMode(self):
        rospy.wait_for_service(self.uav+'/mavros/set_mode')
        try:
            print("Trying offboard", end = '\r')
            flightModeService = rospy.ServiceProxy(self.uav+'/mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='OFFBOARD')
            print("Finished trying offboard", end = '\r')
        except rospy.ServiceException as e:
            print("service set_mode call failed: {}. Offboard Mode could not be set.".format(e))

    def setAltitudeMode(self):
        rospy.wait_for_service(self.uav+'/mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy(self.uav+'/mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='ALTCTL')
        except rospy.ServiceException as e:
            print("service set_mode call failed: {}. Altitude Mode could not be set.".format(e))

    def setPositionMode(self):
        rospy.wait_for_service(self.uav+'/mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy(self.uav+'/mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='POSCTL')
        except rospy.ServiceException as e:
            print("service set_mode call failed: {}. Position Mode could not be set.".format(e))

    def setAutoLandMode(self):
        rospy.wait_for_service(self.uav+'/mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy(self.uav+'/mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='AUTO.LAND')
        except rospy.ServiceException as e:
               print("service set_mode call failed: {}. Autoland Mode could not be set.".format(e))

    def setLoiterMode(self):
        rospy.wait_for_service(self.uav+'/mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy(self.uav+'/mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='AUTO.LOITER')
        except rospy.ServiceException as e:
                print("service set_mode call failed: {}. Loiter Mode could not be set.".format(e))

    def set_param(self, param_id, param_value, trials, set_param_srv):
        """param: PX4 param string, ParamValue, timeout(int): seconds"""
        if param_value.integer != 0:
            value = param_value.integer
        else:
            value = param_value.real
        rospy.loginfo("setting PX4 parameter: {0} with value {1}".
        format(param_id, value))
        # loop_freq = 1  # Hz
        # rate = rospy.Rate(loop_freq)
        # param_set = False
        for i in range(trials):
            try:
                res = set_param_srv(param_id, param_value)
                if res.success:
                    rospy.loginfo("param {0} set to {1} | in {2} of {3} trials".
                    format(param_id, value, i, trials))
                break
            except rospy.ServiceException as e:
                rospy.logerr(e)

            # try:
            #     rate.sleep()
            # except rospy.ROSException as e:
            #     print("Could not call rate.sleep")

        if(not res.success):
            print("failed to set param | param_id: {0}, param_value: {1} | trials: {2}".
            format(param_id, value, trials))


## Drone State callback
class Px4Info():
    def __init__(self):
        self.state = State()
        self.pose = PoseStamped()
    def updateStateCallback(self, msg):
        self.state = msg
    def updatePoseCallback(self, msg):
        self.pose = msg

class VelocityController:
    # initialization method
    def __init__(self):
        # Instantiate a setpoints message
        self.sp = PositionTarget()
        # set the flag to use position setpoints and yaw angle
        self.sp.type_mask = int('010111000111', 2)
        # LOCAL_NED
        self.sp.coordinate_frame = 1
        # initial values for setpoints
        self.max_speed = 1

    ## Update setpoint message
    def updateSp(self, velocity):
        self.sp.velocity.x = velocity[0]
        self.sp.velocity.y = velocity[1]
        self.sp.velocity.z = velocity[2]

class PositionController:
    # initialization method
    def __init__(self):
        # Instantiate a setpoints message
        self.sp = PositionTarget()
        # set the flag to use position setpoints and yaw angle
        self.sp.type_mask = int('010111111000', 2)
        # LOCAL_NED
        self.sp.coordinate_frame = 1

        # initial values for setpoints
        self.sp.position.x = 0.0
        self.sp.position.y = 0.0

        # speed of the drone is set using MPC_XY_CRUISE parameter in MAVLink
        # using QGroundControl. By default it is 5 m/s.

    ## Update setpoint message
    def updateSp(self, pos):
        self.sp.position.x = pos[0]
        self.sp.position.y = pos[1]
        self.sp.position.z = pos[2]


class customPositionController:
    def __init__(self,K, maxSpeed):
        self.K = K
        self.maxSpeed = maxSpeed

    def control(self, curPos, desPos):
        desVel = self.K*(desPos-curPos)
        desVelNorm = np.linalg.norm(desVel)
        if(desVelNorm > self.maxSpeed):
            desVel = desVel/desVelNorm*self.maxSpeed
        return desVel