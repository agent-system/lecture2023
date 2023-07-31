#!/usr/bin/env python

import tf
import rospy
from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.srv import SetModelState, GetModelState  # Added GetModelState import
# Added Quaternion import
from geometry_msgs.msg import Vector3, Twist, Point, Quaternion
import math
import random
from std_srvs.srv import Empty, SetBool, SetBoolResponse
import numpy as np
from rospy import ServiceException
from gazebo_msgs.srv import GetWorldProperties

d = 1.5
global_goal = Point(0, 0, 0.25)
relative_goals = {
    # (Same as original script)
}

k_rep = 2
d_rep = 1.2
k_att = 0.15


def repulsion_force(p, q, axis):
    distance = math.sqrt((p.x - q.x)**2 + (p.y - q.y)**2)
    min_distance = 1  # a small positive number
    distance = max(distance, min_distance)

    if distance <= d_rep:
        if axis == "x":
            force = k_rep * ((p.x - q.x) / distance) / distance**2
        elif axis == "y":
            force = k_rep * ((p.y - q.y) / distance) / distance**2
    else:
        force = 0
    return force


def attraction_force(p, g, axis):
    distance = math.sqrt((p.x - g.x)**2 + (p.y - g.y)**2)
    if axis == "x":
        force = -k_att * ((p.x - g.x) / distance) * distance
    elif axis == "y":
        force = -k_att * ((p.y - g.y) / distance) * distance
    return force


def set_state(model_name, new_position, new_orientation, twist=Twist()):  # Added twist parameter
    rospy.wait_for_service('gazebo/set_model_state')
    try:
        set_state = rospy.ServiceProxy('gazebo/set_model_state', SetModelState)
        model_state = ModelState()
        model_state.model_name = model_name
        model_state.pose.position = new_position
        model_state.pose.orientation = new_orientation
        model_state.twist = twist  # Set the twist
        set_state(model_state)
    except ServiceException as e:  # Changed rospy.ServiceException to ServiceException
        print("Service call failed: %s" % e)


def controller():

    rospy.init_node('object_controller', anonymous=True)

    # Add a new service to control the state of the controller
    rospy.Service('controller_activate', SetBool, service_callback)

    # Prepare the service for setting the state of an object
    rospy.wait_for_service('/gazebo/set_model_state')
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

    last_time = rospy.get_time()
    loop_rate = 100.0
    rate = rospy.Rate(loop_rate)

    while not rospy.is_shutdown():
        states = rospy.wait_for_message("/gazebo/model_states", ModelStates)
        current_time = rospy.get_time()
        dt = current_time - last_time

        global_goal.x += random.uniform(-8, 8) * dt
        global_goal.y += random.uniform(-8, 8) * dt

        for i in range(len(states.name)):
            name = states.name[i]
            if name not in relative_goals:
                continue

            state = states.pose[i]
            desired_state = ModelState()
            desired_state.model_name = name

            goal_position = Point(global_goal.x + relative_goals[name].x,
                                  global_goal.y + relative_goals[name].y,
                                  global_goal.z + relative_goals[name].z)

            total_force_x = 0.0
            total_force_y = 0.0
            for j in range(len(states.name)):
                if i != j:
                    total_force_x += repulsion_force(
                        state.position, states.pose[j].position, "x")
                    total_force_y += repulsion_force(
                        state.position, states.pose[j].position, "y")
            total_force_x += attraction_force(state.position,
                                              goal_position, "x")
            total_force_y += attraction_force(state.position,
                                              goal_position, "y")

            dx = total_force_x * dt
            dy = total_force_y * dt

            desired_state.pose.position.x = state.position.x + dx
            desired_state.pose.position.y = state.position.y + dy
            desired_state.pose.position.z = state.position.z

            res = set_state(desired_state)
            if not res.success:
                rospy.logerr(res.status_message)

        last_time = current_time
        rate.sleep()


if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInterruptException:
        pass
