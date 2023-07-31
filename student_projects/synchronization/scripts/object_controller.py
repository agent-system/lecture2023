#!/usr/bin/env python

import tf
import rospy
from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.srv import SetModelState, GetModelState
from geometry_msgs.msg import Vector3, Twist, Point, Quaternion
import math
import random
from std_srvs.srv import Empty, SetBool, SetBoolResponse
import numpy as np
from rospy import ServiceException
from gazebo_msgs.srv import GetWorldProperties

# Define the distance between objects
d = 2.5

# Define the global goal position
global_goal = Point(6, 6, 0.25)

masses = {
    "object1": 1.0,
    "object2": 1.0,
    "object3": 1.0,
    "object4": 1.0,
    "object5": 1.0,
    "object6": 1.0
}

# Initialize a dictionary to store the velocities of each object
velocities = {
    "object1": Vector3(),
    "object2": Vector3(),
    "object3": Vector3(),
    "object4": Vector3(),
    "object5": Vector3(),
    "object6": Vector3()
}

# Define the relative goal positions for each object
relative_goals = {
    "object1": Point(d, 0, 0),
    "object2": Point(0, -2*d, 0),
    "object3": Point(2*d, -2*d, 0),
    "object4": Point(-d, -4*d, 0),
    "object5": Point(d, -4*d, 0),
    "object6": Point(3*d, -4*d, 0)
}

friction_coefficients = {
    "object1": 0.15,
    "object2": 0.15,
    "object3": 0.15,
    "object4": 0.15,
    "object5": 0.15,
    "object6": 0.15
}

# Potential field parameters
k_rep = 0.5  # Repulsion gain
d_rep = 2.2  # Distance for the repulsion
k_att_1 = 0.04  # Attraction gain
k_att_2 = 0.03  # Attraction gain


def calculate_average_orientation():
    orientations = []
    for object_name in object_names:
        get_state = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)
        resp = get_state(object_name, "")
        current_orientation = resp.pose.orientation
        euler = tf.transformations.euler_from_quaternion(
            [current_orientation.x, current_orientation.y, current_orientation.z, current_orientation.w])
        orientations.append(euler[2])
    return np.mean(orientations)


def check_model_exists(model_name):
    rospy.wait_for_service('gazebo/get_world_properties')
    get_world_properties = rospy.ServiceProxy(
        'gazebo/get_world_properties', GetWorldProperties)
    world_properties = get_world_properties()
    return model_name in world_properties.model_names


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
    except ServiceException as e:
        print("Service call failed: %s" % e)


def quaternion_diff(q1, q2):
    """Calculate the difference between two quaternions"""
    diff = tf.transformations.quaternion_multiply(
        q1, tf.transformations.quaternion_inverse(q2))

    # Calculate the angle difference
    angle_diff = 2 * math.acos(min(1.0, abs(diff[3])))

    # Use quaternion multiplication to determine the direction of rotation
    direction = tf.transformations.quaternion_multiply(diff, [0, 0, 0, 1])
    direction = tf.transformations.quaternion_multiply(
        [0, 0, 0, 1], tf.transformations.quaternion_inverse(diff))

    # If the direction's w component is less than 0, the rotation is in the opposite direction
    if direction[3] < 0:
        angle_diff *= -1

    return angle_diff

# Function to calculate the repulsion force from a point


def repulsion_force(p, q, axis):
    distance = math.sqrt((p.x - q.x)**2 + (p.y - q.y)**2)
    min_distance = 0.6  # a small positive number
    distance = max(distance, min_distance)

    if distance <= d_rep:
        if axis == "x":
            force = k_rep * ((p.x - q.x) / distance) / distance**2
        elif axis == "y":
            force = k_rep * ((p.y - q.y) / distance) / distance**2
    else:
        force = 0
    return force

# Function to calculate the attraction force to a goal


def attraction_force(p, g, axis):
    distance = math.sqrt((p.x - g.x)**2 + (p.y - g.y)**2)
    if axis == "x":
        force = -k_att_1 * ((p.x - g.x) / distance) / distance**2 - k_att_2 * ((p.x - g.x) / distance) * distance
    elif axis == "y":
        force = -k_att_1 * ((p.y - g.y) / distance) / distance**2 - k_att_2 * ((p.y - g.y) / distance) * distance
    return force


class ControllerState:
    def __init__(self):
        self.active = False


# Create a global object to hold the state of the controller
controller_state = ControllerState()

# Create a service callback function


def service_callback(req):
    global controller_state
    controller_state.active = req.data
    return SetBoolResponse(success=True, message="Controller state set successfully.")


def controller():
    rospy.init_node('object_controller', anonymous=True)

    # Add a new service to control the state of the controller
    rospy.Service('controller_activate', SetBool, service_callback)

    # Prepare the service for setting the state of an object
    rospy.wait_for_service('/gazebo/set_model_state')
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

    last_time = rospy.get_time()

    loop_rate = 100.0  # Or whatever value is suitable for your application
    rate = rospy.Rate(loop_rate)

    # Main control loop
    while not rospy.is_shutdown():
        # Get current states
        states = rospy.wait_for_message("/gazebo/model_states", ModelStates)

        current_time = rospy.get_time()
        dt = current_time - last_time

        # Check if the controller is active before performing control actions
        if controller_state.active:
            # # update global goal
            # global_goal.x += random.uniform(-0.5, 0.5) * dt
            # global_goal.y += random.uniform(-0.5, 0.5) * dt

            for i in range(len(states.name)):
                # Get the name of the object
                name = states.name[i]

                if name not in relative_goals:
                    continue

                total_diff = 0  # Sum of quaternion differences
                for j in range(len(states.name)):
                    if i != j:
                        diff = quaternion_diff([states.pose[i].orientation.x, states.pose[i].orientation.y, states.pose[i].orientation.z, states.pose[i].orientation.w], [
                                               states.pose[j].orientation.x, states.pose[j].orientation.y, states.pose[j].orientation.z, states.pose[j].orientation.w])
                        total_diff += diff  # Add the scalar difference to the total

                average_diff = total_diff / (len(states.name) - 1)

                # Get the current state of the object
                state = states.pose[i]

                # Calculate the desired state
                desired_state = ModelState()
                desired_state.model_name = name

                # Calculate the goal position
                goal_position = Point(global_goal.x + relative_goals[name].x,
                                      global_goal.y + relative_goals[name].y,
                                      global_goal.z + relative_goals[name].z)

                # Calculate the forces
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
                

                speed = math.sqrt(velocities[name].x**2 + velocities[name].y**2)
                if speed > 0:
                    friction_force_x = -friction_coefficients[name] * masses[name] * velocities[name].x / speed
                    friction_force_y = -friction_coefficients[name] * masses[name] * velocities[name].y / speed
                else:
                    friction_force_x = 0
                    friction_force_y = 0

                total_force_x += friction_force_x
                total_force_y += friction_force_y

                # Compute the new position based on the total forces
                ax = total_force_x / masses[name]
                ay = total_force_y / masses[name]

                velocities[name].x += ax * dt
                velocities[name].y += ay * dt

                dx = velocities[name].x * dt
                dy = velocities[name].y * dt

                # Update the desired position
                desired_state.pose.position.x = state.position.x + dx
                desired_state.pose.position.y = state.position.y + dy
                desired_state.pose.position.z = state.position.z

                # Calculate the desired orientation
                current_orientation_q = [
                    state.orientation.x, state.orientation.y, state.orientation.z, state.orientation.w]

                ref_q = current_orientation_q

                # Calculate the rotation quaternion based on average_diff
                rotation_q = tf.transformations.quaternion_about_axis(
                    average_diff, (0, 0, 1))

                # Calculate the desired orientation
                desired_orientation_q = tf.transformations.quaternion_multiply(
                    current_orientation_q, rotation_q)
                desired_orientation_q /= np.linalg.norm(desired_orientation_q)

                # Ensure that the current and desired orientations are in the same hemisphere
                if np.dot(desired_orientation_q, ref_q) < 0:
                    desired_orientation_q = -desired_orientation_q

                # Interpolate the current orientation to the desired orientation using SLERP (Spherical Linear Interpolation)
                # Use total_diff to calculate t
                t = 0.02

                desired_orientation_q = tf.transformations.quaternion_slerp(
                    current_orientation_q, desired_orientation_q, t)

                # Update the desired orientation
                desired_state.pose.orientation.x = desired_orientation_q[0]
                desired_state.pose.orientation.y = desired_orientation_q[1]
                desired_state.pose.orientation.z = desired_orientation_q[2]
                desired_state.pose.orientation.w = desired_orientation_q[3]
                desired_state.twist = Twist()

                # Set the new state
                res = set_state(desired_state)

                # If the set_state service failed, print a message
                if not res.success:
                    rospy.logerr(res.status_message)

        last_time = current_time
        rate.sleep()


if __name__ == '__main__':
    try:
        # Initialize ROS node
        rospy.init_node('object_controller', anonymous=True)

        # Wait for the service to become available
        rospy.wait_for_service('/gazebo/set_model_state')
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        # Set the initial state for each object
        for name in relative_goals:
            # Skip if the model does not exist
            if not check_model_exists(name):
                continue

            # Get the current state
            get_state = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)
            state = get_state(name, "")

            # Set a random initial orientation
            theta = random.uniform(0, 2 * math.pi)
            q = tf.transformations.quaternion_from_euler(0, 0, theta)

            # Create a ModelState object
            initial_state = ModelState()
            initial_state.model_name = name
            initial_state.pose.position = state.pose.position
            initial_state.pose.orientation = Quaternion(*q)

            # Set the new state
            set_state(initial_state)

        # Start the controller
        controller()

    except rospy.ROSInterruptException:
        pass
