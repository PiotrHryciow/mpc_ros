#!/usr/bin/env python3.8

import rospy
from geometry_msgs.msg import Twist  # for /cmd_vel
from geometry_msgs.msg import PoseWithCovarianceStamped  # for localization
from geometry_msgs.msg import PoseArray  # for path

from MPC import ModelPredictiveControl
from math_func import euler_from_quaternion
import numpy as np
import pandas as pd


class MPCDriver:
    def __init__(self):
        # load all params
        self.load_params()

        # initialize position[x,y,z] and orientation[roll_x, pitch_y, yaw_z]
        self.position = [0, 0, 0]
        self.orientation = [0, 0, 0]

        # for calculating current velocity
        self.vel = 0

        # prepare message for control (speed and steering)
        self.control_msg = Twist()

        # for path(whole path), and ref(part of path within control horizon)
        self.path = np.array([[0], [0]])
        self.ref = np.array([[0], [0]])

        # initialize mpc class
        self.mpc = ModelPredictiveControl(
            self.control_horizon, self.ts, [self.Lf, self.R], self.constraints
        )

        # go into the controll loop
        self.hz = rospy.Rate(1 / self.ts)

        # to give time for all the information from other topics
        rospy.sleep(2)

        # to save data into csv
        self.save_a = []
        self.save_delta = []
        self.save_x = []
        self.save_y = []
        self.save_psi = []
        self.save_v = []
        self.path_save = []
        self.saved = 0
        self.save_odom_pos = []
        self.save_odom_orient = []

        self.df = pd.DataFrame()
        self.params_df = pd.DataFrame()
        self.path_df = pd.DataFrame()

        # initialize all publishers
        self.publishers_setup()

        # initialize all subscribers
        self.subscribers_setup()

        # for iterating path
        self.i = 0
        while not rospy.is_shutdown():
            # print("xd")

            if not ((self.path.size / 2) < self.control_horizon):

                if (self.path.size / 2) - self.i > self.control_horizon:
                    ref = self.path[:, self.i : self.i + self.control_horizon + 1]
                else:
                    ref = np.hstack((ref[:, 1:], self.path[:, -1:]))

                # states_list(X0) is [pos_x, pos_y, yaw_z, velocity]
                # controls_list[acceleration, steering angle]
                self.controls_list, self.states_list = self.mpc.solve(
                    ref,
                    [self.position[0], self.position[1], self.orientation[2], self.vel],
                )
                # we need to convert from acc to velocity
                self.vel += self.controls_list[0, 0] * self.ts

                # set values for control msg and send it
                self.control_msg.linear.x = self.vel
                self.control_msg.angular.z = self.controls_list[1, 0]

                if self.i > self.path.size / 2 and (
                    (ref[0, 1] - self.position[0]) + (ref[1, 1] - self.position[1])
                    < 0.05
                ):
                    print("arrived")
                    if not self.saved:
                        self.saved = 1
                        self.save_to_csv()

                else:
                    self.control_pub.publish(self.control_msg)
                    self.save_to_list()

                self.i += 1
            # end of controll loop
            # print("xd")
            self.hz.sleep()

    def load_params(self):
        # load mpc params
        # self.prediction_horizon = rospy.get_param('~mpc_params/prediction_horizon', 20)
        self.control_horizon = rospy.get_param("~mpc_params/control_horizon", 10)
        self.ts = rospy.get_param("~mpc_params/time_step", 0.2)
        R = rospy.get_param("~mpc_params/R", 10)
        self.R = np.diag([R])

        # constraints
        self.acc_max = rospy.get_param("~mpc_params/acc_max", 1)
        self.acc_min = rospy.get_param("~mpc_params/acc_min", -0.5)
        self.steering_angles = rospy.get_param("~mpc_params/steering_angles", 0.3)
        self.v_max = rospy.get_param("~mpc_params/v_max", 1)
        self.v_min = rospy.get_param("~mpc_params/v_min", -0.5)
        self.constraints = np.array(
            [
                self.acc_max,
                self.acc_min,
                self.steering_angles,
                self.v_max,
                self.v_min,
            ]
        )

        # load robot params
        self.Lf = rospy.get_param("~robot_params/Lf", 0.1)

        # load ros params
        self.position_topic = rospy.get_param(
            "~ros_params/sub_position_topic", "/odom_combined "
        )
        self.path_topic = rospy.get_param("~ros_params/sub_path_topic", "/set_path ")
        self.control_topic = rospy.get_param(
            "~ros_params/pub_control_topic", "/cmd_vel"
        )

    def subscribers_setup(self):
        self.position_sub = rospy.Subscriber(
            self.position_topic, PoseWithCovarianceStamped, self.position_callback
        )
        self.path_sub = rospy.Subscriber(self.path_topic, PoseArray, self.path_callback)

    def publishers_setup(self):
        self.control_pub = rospy.Publisher(self.control_topic, Twist, queue_size=10)

    def position_callback(self, position: PoseWithCovarianceStamped):
        self.orientation = euler_from_quaternion(
            position.pose.pose.orientation.x,
            position.pose.pose.orientation.y,
            position.pose.pose.orientation.z,
            position.pose.pose.orientation.w,
        )
        # self.orientation = [roll_x, pitch_y, yaw_z]
        self.position = [
            position.pose.pose.position.x,
            position.pose.pose.position.y,
            position.pose.pose.position.z,
        ]
        # print(self.position)
        self.save_odom_orient.append(self.orientation)
        self.save_odom_pos.append(self.position)

        # return x, y, z, roll_x, pitch_y, yaw_z

    def path_callback(self, path_ref: PoseArray):
        path_x = []
        path_y = []
        for pose in path_ref.poses:
            path_x.append(pose.position.x)
            path_y.append(pose.position.y)
        self.path = np.array([path_x, path_y])
        self.i = 0

    def save_to_csv(self):
        self.df["control_a"] = self.save_a
        self.df["control_delta"] = self.save_delta
        self.df["state_x"] = self.save_x
        self.df["state_y"] = self.save_y
        self.df["state_psi"] = self.save_psi
        self.df["state_v"] = self.save_v
        # self.df["state_list"] = self.states_list_save
        self.path_df["path_x"] = self.path[0, :]
        self.path_df["path_y"] = self.path[1, :]
        self.params_df["params"] = [
            self.Lf,
            self.R,
            self.ts,
            self.control_horizon,
            self.constraints,
        ]
        df_save = pd.concat([self.params_df, self.df, self.path_df], axis=1)
        df_save.to_csv("~/mgr/wyniki/tylem2.csv")

    def save_to_list(self):
        # saving each input iteration
        self.save_a.append(self.controls_list[0])
        self.save_delta.append(self.controls_list[1])
        self.save_x.append(self.states_list[0])
        self.save_y.append(self.states_list[1])
        self.save_psi.append(self.states_list[2])
        self.save_v.append(self.states_list[3])


def main():
    rospy.init_node("mpc_driver", anonymous=True)
    mpc = MPCDriver()
    rospy.spin()


if __name__ == "__main__":
    main()
