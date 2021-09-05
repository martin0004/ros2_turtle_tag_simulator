#!/usr/bin/env python3


from geometry_msgs.msg import Twist
import numpy as np
import rclpy
from rclpy.node import Node
from turtle_tag_simulator_interfaces.msg import Turtle
from turtle_tag_simulator_interfaces.msg import Turtles
from turtle_tag_simulator_interfaces.srv import DeleteTurtle
from turtlesim.msg import Pose


class TaggerControlSystemNode(Node):


    def __init__(self):

        super().__init__("tagger_control_system")

        # Note: the turtlesim Pose message contains both pose information (x, y, theta)
        #       AND velocity information (linear_velocity, angular_velocity)

        # Turtle tagger actual pose
        self.actual_pose = Pose()
        self.subscriber_actual_pose_ = self.create_subscription(Pose, "turtle1/pose", self.update_actual_pose_, 10)

        # Turtle tagger target pose
        # - will be pose of another turtle which is "hunted" by the turtle tagger
        self.target_pose = Pose()
        self.timer_target_pose = self.create_timer(0.1, self.update_target_pose_)

        # Names & poses of turtle players currently in the simulation
        self.players_ = Turtles()
        self.subscriber_players_ = self.create_subscription(Turtles, "players", self.update_players_, 10)

        # Check if a player was tagged
        self.timer_ = self.create_timer(0.1, self.check_if_player_tagged_)

        # Command velocity to turtle tagger
        self.publisher_cmd_vel_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.timer_cmd_vel_ = self.create_timer(0.1, self.publish_cmd_vel_)

        # Distance below which the tagger is assumed to have tagged a player
        self.d_tagged = 0.5

        # Angle error below which a linear velocity command is sent to the tagger
        self.e_theta_threshold = 0.25  # rad

        # Gains for P controllers
        self.k_p_theta = 2.0
        self.k_p_d = 2.0

        # Info message
        self.get_logger().info("Turtle tagger control system started.")


    def update_actual_pose_(self, msg):
        """Update internal actual pose of turtle tagger."""

        self.actual_pose = msg


    def update_target_pose_(self):
        """Identifies the pose of the player closest to the tagger."""

        # Initialize target pose and distance to target pose

        target_pose = Pose()

        target_pose.x = self.actual_pose.x
        target_pose.y = self.actual_pose.y
        target_pose.theta = 0.0 # Placeholder value - target pose theta not used by control system

        d_target_pose = np.Inf

        # Update target pose with pose of closest player turtle

        for player in self.players_.turtles:

            d = ( (self.actual_pose.x-player.x)**2 + (self.actual_pose.y-player.y)**2  )**0.5

            if d < d_target_pose:

                # Target pose x and y values
                target_pose.x = player.x
                target_pose.y = player.y

                # Target pose theta value
                # (set equal to the angle would have if it were at its actual x-y
                # location and rotated to point at the target location)

                delta_x = target_pose.x - self.actual_pose.x
                delta_y = target_pose.y - self.actual_pose.y
                target_pose.theta = np.arctan2(delta_y, delta_x) 
                
                # Updated distance to target pose
                d_target_pose = d
        
        self.target_pose = target_pose


    def update_players_(self, msg):
        """Update internal list of turler players currently in the simulator."""

        self.players_ = msg


    def check_if_player_tagged_(self):
        """Checks if the tagger is close enough of a player to tag it."""

        for player in self.players_.turtles:

            d = ( (self.actual_pose.x - player.x)**2 + (self.actual_pose.y - player.y)**2 )**0.5

            # If tagger close enough of player, player assumed tagged and deleted.

            if d < self.d_tagged:
                client = self.create_client(DeleteTurtle, "delete_turtle")

                while not client.wait_for_service(1.0):
                    self.get_logger().warn("Waiting for delete_turtle service...")

                request = DeleteTurtle.Request()
                request.name = player.name

                client.call_async(request)
                # No need to manipulate response, so no future object.



    def publish_cmd_vel_(self):
        """Derive and publish command velocity for turtle tagger."""

        cmd_vel = Twist()

        # Pose angle error
        e_theta = self.target_pose.theta - self.actual_pose.theta;

        # Correction to keep only shortest rotation angle
        if e_theta < -np.pi:
            e_theta += 2*np.pi
        elif e_theta > np.pi:
            e_theta -= 2*np.pi

        # Angular velocity command (P controller)
        # - command only on z since this is a 2D simulation
        cmd_vel.angular.z = self.k_p_theta * e_theta

        # Linear velocity command (P controller)
        # - turtlesim linear command velocities are expressed in the turtle body frame
        #   therefore we only derive velocity x

        d = ( (self.target_pose.x - self.actual_pose.x)**2 + (self.target_pose.y - self.actual_pose.y)**2 )**0.5
        cmd_vel.linear.x = self.k_p_d * d

        # Step function (set linear velocity command to 0 if angle error high)
        if abs(e_theta) >= self.e_theta_threshold:
            cmd_vel.linear.x = 0.0

        # Publish command velocity
        self.publisher_cmd_vel_.publish(cmd_vel)


def main(args=None):

    rclpy.init(args=args)
    node = TaggerControlSystemNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":

    main()


