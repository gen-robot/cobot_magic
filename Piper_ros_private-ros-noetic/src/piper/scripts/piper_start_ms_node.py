#!/usr/bin/env python3
# -*-coding:utf8-*-
# 本文件为同时打开主从臂的节点，通过mode参数控制是读取还是控制
# 默认认为从臂有夹爪
# mode为1时为发送主从臂消息，
# mode为0时为控制从臂，不发送主臂消息，此时如果要控制从臂，需要给主臂的topic发送消息
from typing import (
    Optional,
)
import rospy
import rosnode
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import time
import threading
import argparse
from piper_sdk import *
from piper_sdk import C_PiperInterface
from piper_msgs.msg import PiperStatusMsg, PosCmd
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler  # 用于欧拉角到四元数的转换

def check_ros_master():
    try:
        rosnode.rosnode_ping('rosout', max_count=1, verbose=False)
        rospy.loginfo("ROS Master is running.")
    except rosnode.ROSNodeIOException:
        rospy.logerr("ROS Master is not running.")
        raise RuntimeError("ROS Master is not running.")

class C_PiperRosNode():
    """机械臂ros节点
    """
    def __init__(self) -> None:
        check_ros_master()
        rospy.init_node('piper_start_all_node', anonymous=True)

        self.can_port = "can0"
        if rospy.has_param('~can_port'):
            self.can_port = rospy.get_param("~can_port")
            rospy.loginfo("%s is %s", rospy.resolve_name('~can_port'), self.can_port)
        else: 
            rospy.loginfo("未找到can_port参数,请输入 _can_port:=can0 类似的格式")
            exit(0)
        # 模式，模式为1的时候，才能够控制从臂
        self.mode = 0
        if rospy.has_param('~mode'):
            self.mode = rospy.get_param("~mode")
            rospy.loginfo("%s is %s", rospy.resolve_name('~mode'), self.mode)
        else:
            rospy.loginfo("未找到mode参数,请输入 _mode:=0 类似的格式")
            exit(0)

        # 是否自动使能，默认不自动使能，只有模式为0的时候才能够被设置为自动使能
        self.auto_enable = False
        if rospy.has_param('~auto_enable'):
            if(rospy.get_param("~auto_enable") and self.mode == 1):
                self.auto_enable = True
        rospy.loginfo("%s is %s", rospy.resolve_name('~auto_enable'), self.auto_enable)
        # publish
        self.joint_std_pub_puppet = rospy.Publisher('/puppet/joint_states', JointState, queue_size=1)
        # 默认模式为0，读取主从臂消息
        if(self.mode == 0):
            self.joint_std_pub_master = rospy.Publisher('/master/joint_states', JointState, queue_size=1)
        self.arm_status_pub = rospy.Publisher('/puppet/arm_status', PiperStatusMsg, queue_size=1)
        self.end_pose_pub = rospy.Publisher('/puppet/end_pose', Pose, queue_size=1)
        
        self.__enable_flag = False
        # 从臂消息
        self.joint_state_slave = JointState()
        self.joint_state_slave.name = ['joint0', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self.joint_state_slave.position = [0.0] * 7
        self.joint_state_slave.velocity = [0.0] * 7
        self.joint_state_slave.effort = [0.0] * 7
        # 主臂消息
        self.joint_state_master = JointState()
        self.joint_state_master.name = ['joint0', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self.joint_state_master.position = [0.0] * 7
        self.joint_state_master.velocity = [0.0] * 7
        self.joint_state_master.effort = [0.0] * 7

        self.piper = C_PiperInterface(can_name=self.can_port)
        self.piper.ConnectPort()
        # 模式为1的时候，订阅控制消息
        if(self.mode == 1):
            sub_pos_th = threading.Thread(target=self.SubPosThread)
            sub_joint_th = threading.Thread(target=self.SubJointThread)
            sub_enable_th = threading.Thread(target=self.SubEnableThread)
            
            sub_pos_th.daemon = True
            sub_joint_th.daemon = True
            sub_enable_th.daemon = True
            
            sub_pos_th.start()
            sub_joint_th.start()
            sub_enable_th.start()

    def GetEnableFlag(self):
        return self.__enable_flag

    def Pubilsh(self):
        """机械臂消息发布
        """
        rate = rospy.Rate(200)  # 200 Hz
        enable_flag = False
        # 设置超时时间（秒）
        timeout = 5
        # 记录进入循环前的时间
        start_time = time.time()
        elapsed_time_flag = False
        while not rospy.is_shutdown():
            # print(self.piper.GetArmJointGripperCtrlMsgs())
            # print(self.piper.GetArmEndPoseMsgs())
            # print(self.piper.GetArmJointGripperMsgs())
            # print(self.piper.GetArmLowSpdInfoMsgs())
            # print(self.piper.GetArmStatus())
            # print(self.piper.GetArmJointGripperMsgs())
            # self.piper.ArmParamEnquiryAndConfig(1,0,2,0,3)
            # self.piper.MasterSlaveConfig(0x00, 0x00, 0x00, 0x00)
            # self.piper.SearchMotorMaxAngleSpdAccLimit(6,0x01)
            if(self.auto_enable and self.mode == 1):
                while not (enable_flag):
                    elapsed_time = time.time() - start_time
                    print("--------------------")
                    enable_flag = self.piper.GetArmLowSpdInfoMsgs().motor_1.foc_status.driver_enable_status and \
                        self.piper.GetArmLowSpdInfoMsgs().motor_2.foc_status.driver_enable_status and \
                        self.piper.GetArmLowSpdInfoMsgs().motor_3.foc_status.driver_enable_status and \
                        self.piper.GetArmLowSpdInfoMsgs().motor_4.foc_status.driver_enable_status and \
                        self.piper.GetArmLowSpdInfoMsgs().motor_5.foc_status.driver_enable_status and \
                        self.piper.GetArmLowSpdInfoMsgs().motor_6.foc_status.driver_enable_status
                    print("使能状态:",enable_flag)
                    if(enable_flag):
                        self.__enable_flag = True
                    self.piper.EnableArm(7)
                    self.piper.GripperCtrl(0,1000,0x01, 0)
                    print("--------------------")
                    # 检查是否超过超时时间
                    if elapsed_time > timeout:
                        print("超时....")
                        elapsed_time_flag = True
                        enable_flag = True
                        break
                    time.sleep(1)
                    pass
            if(elapsed_time_flag):
                print("程序自动使能超时,退出程序")
                exit(0)
            # 发布消息
            self.PublishSlaveArmJointAndGripper()
            self.PublishSlaveArmState()
            self.PublishSlaveArmEndPose()
            # 模式为0的时候，发布主臂消息
            if(self.mode == 0):
                self.PublishMasterArmJointAndGripper()

            rate.sleep()
    
    def PublishSlaveArmState(self):
        arm_status = PiperStatusMsg()
        arm_status.ctrl_mode = self.piper.GetArmStatus().arm_status.ctrl_mode
        arm_status.arm_status = self.piper.GetArmStatus().arm_status.arm_status
        arm_status.mode_feedback = self.piper.GetArmStatus().arm_status.mode_feed
        arm_status.teach_status = self.piper.GetArmStatus().arm_status.teach_status
        arm_status.motion_status = self.piper.GetArmStatus().arm_status.motion_status
        arm_status.trajectory_num = self.piper.GetArmStatus().arm_status.trajectory_num
        arm_status.err_code = self.piper.GetArmStatus().arm_status.err_code
        arm_status.joint_1_angle_limit = self.piper.GetArmStatus().arm_status.err_status.joint_1_angle_limit
        arm_status.joint_2_angle_limit = self.piper.GetArmStatus().arm_status.err_status.joint_2_angle_limit
        arm_status.joint_3_angle_limit = self.piper.GetArmStatus().arm_status.err_status.joint_3_angle_limit
        arm_status.joint_4_angle_limit = self.piper.GetArmStatus().arm_status.err_status.joint_4_angle_limit
        arm_status.joint_5_angle_limit = self.piper.GetArmStatus().arm_status.err_status.joint_5_angle_limit
        arm_status.joint_6_angle_limit = self.piper.GetArmStatus().arm_status.err_status.joint_6_angle_limit
        arm_status.communication_status_joint_1 = self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_1
        arm_status.communication_status_joint_2 = self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_2
        arm_status.communication_status_joint_3 = self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_3
        arm_status.communication_status_joint_4 = self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_4
        arm_status.communication_status_joint_5 = self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_5
        arm_status.communication_status_joint_6 = self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_6
        self.arm_status_pub.publish(arm_status)
    
    def PublishSlaveArmEndPose(self):
        # 末端位姿
        endpos = Pose()
        endpos.position.x = self.piper.ArmEndPose.end_pose.X_axis/1000000
        endpos.position.y = self.piper.ArmEndPose.end_pose.Y_axis/1000000
        endpos.position.z = self.piper.ArmEndPose.end_pose.Z_axis/1000000
        roll = self.piper.ArmEndPose.end_pose.RX_axis/1000
        pitch = self.piper.ArmEndPose.end_pose.RY_axis/1000
        yaw = self.piper.ArmEndPose.end_pose.RZ_axis/1000
        quaternion = quaternion_from_euler(roll, pitch, yaw)
        endpos.orientation.x = quaternion[0]
        endpos.orientation.y = quaternion[1]
        endpos.orientation.z = quaternion[2]
        endpos.orientation.w = quaternion[3]
        self.end_pose_pub.publish(endpos)
    
    def PublishSlaveArmJointAndGripper(self):
        # 从臂反馈消息
        self.joint_state_slave.header.stamp = rospy.Time.now()
        joint_0:float = (self.piper.GetArmJointMsgs().joint_state.joint_1/1000) * 0.017444
        joint_1:float = (self.piper.GetArmJointMsgs().joint_state.joint_2/1000) * 0.017444
        joint_2:float = (self.piper.GetArmJointMsgs().joint_state.joint_3/1000) * 0.017444
        joint_3:float = (self.piper.GetArmJointMsgs().joint_state.joint_4/1000) * 0.017444
        joint_4:float = (self.piper.GetArmJointMsgs().joint_state.joint_5/1000) * 0.017444
        joint_5:float = (self.piper.GetArmJointMsgs().joint_state.joint_6/1000) * 0.017444
        joint_6:float = self.piper.GetArmGripperMsgs().gripper_state.grippers_angle/1000000
        vel_0:float = self.piper.GetArmHighSpdInfoMsgs().motor_1.motor_speed/1000
        vel_1:float = self.piper.GetArmHighSpdInfoMsgs().motor_2.motor_speed/1000
        vel_2:float = self.piper.GetArmHighSpdInfoMsgs().motor_3.motor_speed/1000
        vel_3:float = self.piper.GetArmHighSpdInfoMsgs().motor_4.motor_speed/1000
        vel_4:float = self.piper.GetArmHighSpdInfoMsgs().motor_5.motor_speed/1000
        vel_5:float = self.piper.GetArmHighSpdInfoMsgs().motor_6.motor_speed/1000
        effort_6:float = self.piper.GetArmGripperMsgs().gripper_state.grippers_effort/1000
        self.joint_state_slave.position = [joint_0,joint_1, joint_2, joint_3, joint_4, joint_5,joint_6]  # Example values
        self.joint_state_slave.velocity = [vel_0, vel_1, vel_2, vel_3, vel_4, vel_5, 0.0]  # Example values
        self.joint_state_slave.effort[6] = effort_6
        self.joint_std_pub_puppet.publish(self.joint_state_slave)
    
    def PublishMasterArmJointAndGripper(self):
        # 主臂控制消息
        self.joint_state_master.header.stamp = rospy.Time.now()
        joint_0:float = (self.piper.GetArmJointCtrl().joint_ctrl.joint_1/1000) * 0.017444
        joint_1:float = (self.piper.GetArmJointCtrl().joint_ctrl.joint_2/1000) * 0.017444
        joint_2:float = (self.piper.GetArmJointCtrl().joint_ctrl.joint_3/1000) * 0.017444
        joint_3:float = (self.piper.GetArmJointCtrl().joint_ctrl.joint_4/1000) * 0.017444
        joint_4:float = (self.piper.GetArmJointCtrl().joint_ctrl.joint_5/1000) * 0.017444
        joint_5:float = (self.piper.GetArmJointCtrl().joint_ctrl.joint_6/1000) * 0.017444
        joint_6:float = self.piper.GetArmGripperCtrl().gripper_ctrl.grippers_angle/1000000
        self.joint_state_master.position = [joint_0,joint_1, joint_2, joint_3, joint_4, joint_5,joint_6]  # Example values
        self.joint_std_pub_master.publish(self.joint_state_master)
    
    def SubPosThread(self):
        """机械臂末端位姿订阅
        
        """
        rospy.Subscriber('pos_cmd', PosCmd, self.pos_callback)
        rospy.spin()
    
    def SubJointThread(self):
        """机械臂关节订阅
        
        """
        rospy.Subscriber('/master/joint_states', JointState, self.joint_callback)
        rospy.spin()
    
    def SubEnableThread(self):
        """机械臂使能
        
        """
        rospy.Subscriber('/enable_flag', Bool, self.enable_callback)
        rospy.spin()

    def pos_callback(self, pos_data):
        """机械臂末端位姿订阅回调函数

        Args:
            pos_data (): 
        """
        rospy.loginfo("Received PosCmd:")
        rospy.loginfo("x: %f", pos_data.x)
        rospy.loginfo("y: %f", pos_data.y)
        rospy.loginfo("z: %f", pos_data.z)
        rospy.loginfo("roll: %f", pos_data.roll)
        rospy.loginfo("pitch: %f", pos_data.pitch)
        rospy.loginfo("yaw: %f", pos_data.yaw)
        rospy.loginfo("gripper: %f", pos_data.gripper)
        rospy.loginfo("mode1: %d", pos_data.mode1)
        rospy.loginfo("mode2: %d", pos_data.mode2)
        x = round(pos_data.x*1000)
        y = round(pos_data.y*1000)
        z = round(pos_data.z*1000)
        rx = round(pos_data.roll*1000)
        ry = round(pos_data.pitch*1000)
        rz = round(pos_data.yaw*1000)
        if(self.GetEnableFlag()):
            self.piper.MotionCtrl_1(0x00, 0x00, 0x00)
            self.piper.MotionCtrl_2(0x01, 0x02, 50)
            self.piper.EndPoseCtrl(x, y, z, 
                                    rx, ry, rz)
            gripper = round(pos_data.gripper*1000*1000)
            if(pos_data.gripper>80000): gripper = 80000
            if(pos_data.gripper<0): gripper = 0
            if(self.girpper_exist):
                self.piper.GripperCtrl(abs(gripper), 1000, 0x01, 0)
            self.piper.MotionCtrl_2(0x01, 0x00, 50)
    
    def joint_callback(self, joint_data):
        """机械臂关节角回调函数

        Args:
            joint_data (): 
        """
        factor = 57324.840764 #1000*180/3.14
        factor1 = 57.32484
        rospy.loginfo("Received Joint States:")
        rospy.loginfo("joint_0: %f", joint_data.position[0]*1)
        rospy.loginfo("joint_1: %f", joint_data.position[1]*1)
        rospy.loginfo("joint_2: %f", joint_data.position[2]*1)
        rospy.loginfo("joint_3: %f", joint_data.position[3]*1)
        rospy.loginfo("joint_4: %f", joint_data.position[4]*1)
        rospy.loginfo("joint_5: %f", joint_data.position[5]*1)
        rospy.loginfo("joint_6: %f", joint_data.position[6]*1)
        joint_0 = round(joint_data.position[0]*factor)
        joint_1 = round(joint_data.position[1]*factor)
        joint_2 = round(joint_data.position[2]*factor)
        joint_3 = round(joint_data.position[3]*factor)
        joint_4 = round(joint_data.position[4]*factor)
        joint_5 = round(joint_data.position[5]*factor)
        joint_6 = round(joint_data.position[6]*1000*1000)
        if(joint_6>80000): joint_6 = 80000
        if(joint_6<0): joint_6 = 0
        if(self.GetEnableFlag()):
            self.piper.MotionCtrl_2(0x01, 0x01, 100)
            self.piper.JointCtrl(joint_0, joint_1, joint_2, 
                                    joint_3, joint_4, joint_5)
            self.piper.GripperCtrl(abs(joint_6), 1000, 0x01, 0)
            self.piper.MotionCtrl_2(0x01, 0x01, 100)
            pass
    
    def enable_callback(self, enable_flag:Bool):
        """机械臂使能回调函数

        Args:
            enable_flag (): 
        """
        rospy.loginfo("Received enable flag:")
        rospy.loginfo("enable_flag: %s", enable_flag.data)
        if(enable_flag.data):
            self.__enable_flag = True
            self.piper.EnableArm(7)
            self.piper.GripperCtrl(0,1000,0x01, 0)
        else:
            self.__enable_flag = False
            self.piper.DisableArm(7)
            self.piper.GripperCtrl(0,1000,0x00, 0)

if __name__ == '__main__':
    try:
        piper_ms = C_PiperRosNode()
        piper_ms.Pubilsh()
    except rospy.ROSInterruptException:
        pass

