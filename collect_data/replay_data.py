#coding=utf-8
import os
import numpy as np
import cv2
import h5py
import argparse
import rospy
import time
from functools import partial

import matplotlib.pyplot as plt

from cv_bridge import CvBridge
from std_msgs.msg import Header
from sensor_msgs.msg import Image, JointState
from geometry_msgs.msg import Twist

import sys
sys.path.append("./")
def load_hdf5(hdf5_path):
    dataset_path = os.path.join(hdf5_path)
    if not os.path.isfile(dataset_path):
        print(f'Dataset does not exist at \n{dataset_path}\n')
        exit()

    with h5py.File(dataset_path, 'r') as root:
        is_sim = root.attrs['sim']
        compressed = root.attrs.get('compress', True)
        qpos = root['/observations/qpos'][()]
        qvel = root['/observations/qvel'][()]
        if 'effort' in root.keys():
            effort = root['/observations/effort'][()]
        else:
            effort = None
        action = root['/action'][()]
        base_action = root['/base_action'][()]
        
        image_dict = dict()
        for cam_name in root[f'/observations/images/'].keys():
            image_dict[cam_name] = root[f'/observations/images/{cam_name}'][()]
        
        # if compressed:
            # compress_len = root['/compress_len'][()]

    if compressed:
        for cam_id, cam_name in enumerate(image_dict.keys()):
            # un-pad and uncompress
            padded_compressed_image_list = image_dict[cam_name]
            image_list = []
            for frame_id, padded_compressed_image in enumerate(padded_compressed_image_list): # [:1000] to save memory
                # image_len = int(compress_len[cam_id, frame_id])
                
                compressed_image = padded_compressed_image
                image = cv2.imdecode(np.frombuffer(compressed_image, np.uint8), cv2.IMREAD_COLOR)
                image_list.append(image)
            image_dict[cam_name] = image_list

        # import pdb; pdb.set_trace()

    return qpos, qvel, effort, action, base_action, image_dict

def print_msg(msg, name):
    print(name, msg.position)

def main(args):
    rospy.init_node("replay_node")

    puppet_arm_left_publisher = rospy.Publisher(args.puppet_arm_left_topic, JointState, queue_size=10)
    puppet_arm_right_publisher = rospy.Publisher(args.puppet_arm_right_topic, JointState, queue_size=10)
    
    master_arm_left_publisher = rospy.Publisher(args.master_arm_left_topic, JointState, queue_size=10)
    master_arm_right_publisher = rospy.Publisher(args.master_arm_right_topic, JointState, queue_size=10)
    
    robot_base_publisher = rospy.Publisher(args.robot_base_topic, Twist, queue_size=10)


    origin_left = [-0.0057,-0.031, -0.0122, -0.032, 0.0099, 0.0179, 0.2279]  
    origin_right = [ 0.0616, 0.0021, 0.0475, -0.1013, 0.1097, 0.0872, 0.2279]

    left0 = [-0.00133514404296875, 0.00209808349609375, 0.01583099365234375, -0.032616615295410156, -0.00286102294921875, 0.00095367431640625, 0.0]
    right0 = [-0.00133514404296875, 0.00438690185546875, 0.034523963928222656, -0.053597450256347656, -0.00476837158203125, -0.00209808349609375, 3.557830810546875]
    left1 = [-0.00133514404296875, 0.00209808349609375, 0.01583099365234375, -0.032616615295410156, -0.00286102294921875, 0.00095367431640625, -0.3393220901489258]
    right1 = [-0.00133514404296875, 0.00247955322265625, 0.01583099365234375, -0.032616615295410156, -0.00286102294921875, 0.00095367431640625, -0.3397035598754883]

    
    joint_state_msg = JointState()
    joint_state_msg.header =  Header()
    joint_state_msg.name = ['joint0', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']  # 设置关节名称
    twist_msg = Twist()

    rate = rospy.Rate(args.frame_rate)

    # rospy.Subscriber(args.puppet_arm_left_topic, JointState, partial(print_msg, name="left:"), queue_size=1000, tcp_nodelay=True)
    # rospy.Subscriber(args.puppet_arm_right_topic, JointState, partial(print_msg, name="right:"), queue_size=1000, tcp_nodelay=True)
    
    # qposs, qvels, efforts, actions, base_actions, image_dicts = load_hdf5(args.hdf5_path)

    # fig = plt.figure(figsize=(70, 20))
    
    # for i in range(actions.shape[-1]):
    #    plt.subplot(2, 7, i+1)
    #     plt.plot(range(actions.shape[0]), actions[:, i])
    #plt.savefig("tst.png")
    #plt.tight_layout()
    #exit()


    last_action = [-0.0057,-0.031, -0.0122, -0.032, 0.0099, 0.0179, 0.2279, 0.0616, 0.0021, 0.0475, -0.1013, 0.1097, 0.0872, 0.2279]
    rate = rospy.Rate(100)
    # for action in actions:
    for i in range(100):
        if(rospy.is_shutdown()):
            print("BBBBBB")
            break
        
        # new_actions = np.linspace(last_action, last_action, 20) # 插值
        # # last_action = last_action
        # for act in new_actions:
        #     # print(np.round(act[:7], 4))
        cur_timestamp = rospy.Time.now()  # 设置时间戳
        joint_state_msg.header.stamp = cur_timestamp 
        
        joint_state_msg.position = origin_left #last_action[:7]
        master_arm_left_publisher.publish(joint_state_msg)

        joint_state_msg.position = origin_left #last_action[7:]
        master_arm_right_publisher.publish(joint_state_msg)   

        if(rospy.is_shutdown()):
            print("CCCCCCC")
            break
        rate.sleep() 


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--hdf5_path', action='store', type=str, help='Dataset_dir.',
                        default=None, required=False)

    parser.add_argument('--episode_idx', action='store', type=int, help='Episode index.',default=0, required=False)
    
    parser.add_argument('--camera_names', action='store', type=str, help='camera_names',
                        default=['cam_high', 'cam_left_wrist', 'cam_right_wrist'], required=False)
    
    parser.add_argument('--img_front_topic', action='store', type=str, help='img_front_topic',
                        default='/camera_f/color/image_raw', required=False)
    parser.add_argument('--img_left_topic', action='store', type=str, help='img_left_topic',
                        default='/camera_l/color/image_raw', required=False)
    parser.add_argument('--img_right_topic', action='store', type=str, help='img_right_topic',
                        default='/camera_r/color/image_raw', required=False)
    
    parser.add_argument('--master_arm_left_topic', action='store', type=str, help='master_arm_left_topic',
                        default='/master/joint_left', required=False)
    parser.add_argument('--master_arm_right_topic', action='store', type=str, help='master_arm_right_topic',
                        default='/master/joint_right', required=False)
    
    parser.add_argument('--puppet_arm_left_topic', action='store', type=str, help='puppet_arm_left_topic',
                        default='/puppet/joint_left', required=False)
    parser.add_argument('--puppet_arm_right_topic', action='store', type=str, help='puppet_arm_right_topic',
                        default='/puppet/joint_right', required=False)
    
    parser.add_argument('--robot_base_topic', action='store', type=str, help='robot_base_topic',
                        default='/cmd_vel', required=False)
    parser.add_argument('--use_robot_base', action='store', type=bool, help='use_robot_base',
                        default=False, required=False)
    
    parser.add_argument('--frame_rate', action='store', type=int, help='frame_rate',
                        default=30, required=False)
    
    parser.add_argument('--only_pub_master', action='store_true', help='only_pub_master',required=False)
    
    

    args = parser.parse_args()
    main(args)
    # python collect_data.py --max_timesteps 500 --is_compress --episode_idx 0 
