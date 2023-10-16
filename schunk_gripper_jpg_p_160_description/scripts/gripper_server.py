#!/usr/bin/env python3

import rospy
import numpy
import sys
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from threading import Thread
from pybullet_utils.srv import ChangeControlMode
from schunk_gripper_jpg_p_160_description.srv import MoveGripper


def green_p(msg):
    print('\033[92m' + msg + '\033[0m')


def red_p(msg):
    print('\033[91m' + msg + '\033[0m')


def blue_p(msg):
    print('\033[94m' + msg + '\033[0m')


def yellow_p(msg):
    print('\033[93m' + msg + '\033[0m')


def cyan_p(msg):
    print('\033[96m' + msg + '\033[0m')


def move_gripper(srv, controlled_joint_name, target_position, joint_target_publish_rate, real_js, js_topic_name):
    joint_min_val = 0.0125
    joint_max_val = 0.34

    target_velocity = (joint_max_val - joint_min_val) / 2

    control_mode = srv.control_mode

    if (control_mode != 'open' and control_mode != 'close'):
        red_p('Wrong control mode')
        return 'false'
    if (control_mode == 'open'):
        print('Mode: open.')

    if (control_mode == 'close'):
        print('Mode: close.')

    print('Wait for ' + js_topic_name)
    initial_joint_state = rospy.wait_for_message(js_topic_name, JointState)
    print(js_topic_name + ' message recived')

    target_position[0] = initial_joint_state.position[initial_joint_state.name.index(controlled_joint_name)]

    finish = False
    if (control_mode == 'close'):
        dx = -abs(target_velocity) / joint_target_publish_rate
    elif (control_mode == 'open'):
        dx = abs(target_velocity) / joint_target_publish_rate

    rate = rospy.Rate(joint_target_publish_rate)
    print('dx: ' + str(dx))
    while not finish:
        target_position[0] += dx
        real_position = real_js[0].position[real_js[0].name.index(controlled_joint_name)]
        print("Joint_states: " + str(real_position))

        print("target_position: " + str(target_position[0]))
        if ((real_position < joint_min_val and dx < 0) or (real_position > joint_max_val and dx > 0)):
            finish = True
            print('Joint limit: ' + str(real_position))
#            target_position[0] = real_position
        rate.sleep()
    return 'true'


def jointStateSubscriber(data, args):
    real_js = args[0]
    real_js[0] = data


def jointTargetPublisher(jt_pub, joint_target_publish_rate, controlled_joint_name, target_position):
    rate = rospy.Rate(joint_target_publish_rate)
    jt_msg = JointState()

    while not rospy.is_shutdown():
        jt_msg.header = Header()
        jt_msg.header.stamp = rospy.Time.now()
        jt_msg.name = [controlled_joint_name]
        jt_msg.position = target_position
        jt_msg.velocity = [0]
        jt_msg.effort = [0]
        jt_pub.publish(jt_msg)
        rate.sleep()


def main(my_argv):
    print(my_argv)
    print(len(my_argv))
    if (len(my_argv) == 5):
        node_name = my_argv[3].replace('__name:=', '')
        robot_name = my_argv[1]
        gripper_name = my_argv[2]
        controlled_joint_name = 'left_joint_circle'
    elif (len(my_argv) == 6):
        node_name = my_argv[3].replace('__name:=', '')
        robot_name = my_argv[1]
        gripper_name = my_argv[2]
        controlled_joint_prefix = my_argv[3]
        controlled_joint_name = controlled_joint_prefix + 'left_joint_circle'
    else:
        print('No input params, it is required robot_name, gripper_name and controlled_joint_prefix')
        node_name = 'schunk_gripper_jpg_p_160_server'
        robot_name =  'schunk_gripper_jpg_p_160'
        gripper_name = 'schunk_gripper_jpg_p_160'
        controlled_joint_name = 'left_joint_circle'

    print('node_name = ' + node_name)
    print('robot_name = ' + robot_name)
    print('gripper_name = ' + gripper_name)
    print('controlled_joint_name = ' + controlled_joint_name)

    rospy.init_node(node_name)

    if (rospy.has_param('/pybullet_simulation/joint_target_publish_rate')):
        joint_target_publish_rate = rospy.get_param('/pybullet_simulation/joint_target_publish_rate')
    else:
        red_p('/pybullet_simulation/joint_target_publish_rate pasam not set')
        raise SystemExit

    print('Waiting for /joint_states topic')

    real_js = []

    try:
        real_js = [rospy.wait_for_message('/joint_states', JointState, timeout=2)]
        js_topic_name = '/joint_states'
    except (rospy.ROSException):
        print("/joint_states doesn't recived")
        print('Waiting for /' + robot_name + '/joint_states topic')
        try:
            real_js = [rospy.wait_for_message('/' + robot_name + '/joint_states', JointState, timeout=2)]
            js_topic_name = '/' + robot_name + '/joint_states'
        except (rospy.ROSException):
            red_p("/" + robot_name + "/joint_states doesn't recived")
            exit(0)
        except (rospy.ROSInterruptException):
            red_p("ROSInterruptException")
            exit(0)
    except (rospy.ROSInterruptException):
        red_p("ROSInterruptException")
        exit(0)



    position = [ real_js[0].position[real_js[0].name.index(controlled_joint_name)] ]

    rospy.Subscriber(js_topic_name, JointState, jointStateSubscriber, (real_js, ))

    jt_pub = rospy.Publisher('/' + gripper_name + '/joint_target', JointState, queue_size=1)

    jt_pub_thread = Thread(target=jointTargetPublisher, args=(jt_pub, joint_target_publish_rate, controlled_joint_name, position))
    jt_pub_thread.start()

    rospy.Service("/move_" + gripper_name, MoveGripper, lambda msg: move_gripper(msg, controlled_joint_name, position, joint_target_publish_rate, real_js, js_topic_name))

    print("Service /move_" + gripper_name + " started")

    rospy.spin()
    jt_pub_thread.join()


if __name__ == '__main__':
    main(sys.argv)
