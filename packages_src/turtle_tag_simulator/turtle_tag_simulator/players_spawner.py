#!/usr/bin/env python3

from functools import partial
import numpy as np
import rclpy
from rclpy.node import Node
from turtlesim.srv import Kill
from turtlesim.srv import Spawn
from turtle_tag_simulator_interfaces.msg import Turtle
from turtle_tag_simulator_interfaces.msg import Turtles
from turtle_tag_simulator_interfaces.srv import DeleteTurtle

TURTLESIM_WINDOW_WIDTH = 11.0
TURTLESIM_WINDOW_HEIGHT = 11.0


class PlayersSpawnerNode(Node):


    def __init__(self):

        super().__init__("players_spawner")

        # Names & poses of turtle players currently in the simulation
        # (list of Turtle messages)
        self.players_ = []

        # Total number of turtle players which were spawned, no matter if they
        # are still in the simulation or not.
        self.players_counter_ = 0

        # Publisher of current turtle player list
        self.publisher_players_ = self.create_publisher(Turtles, "players", 10)
        self.timer_publish_players_ = self.create_timer(0.1, self.publish_players_)

        # Spawn new players.
        self.create_timer(2.0, self.spawn_player_)

        # Delete tagged turtles.
        self.server_delete_player = self.create_service(DeleteTurtle, "/delete_turtle", self.delete_player_) 

        # Info message
        self.get_logger().info("Started spawning players.")


    def publish_players_(self):
        """Publish list of players currently in the simulation."""

        msg = Turtles()
        msg.turtles = self.players_
        self.publisher_players_.publish(msg)


    def spawn_player_(self):
        """Add a new player in the simulation."""

        client = self.create_client(Spawn, "/spawn")

        if not client.service_is_ready:

            self.get_logger().info("Spawning service not ready yet...")
        
        else:

            # Initialize request for spawning new turtle
            request = Spawn.Request()
 
            # Generate random location for a new turtle
            #
            # Note: - the turtlesim simulator window size is about 11 units x 11 units
            #       - the origin is the lower left corner of the window
            #       - x axis points left
            #       - y axis points up
            #
            request.x = np.random.default_rng().uniform(low = 0.0, high = TURTLESIM_WINDOW_WIDTH) 
            request.y = np.random.default_rng().uniform(low = 0.0, high = TURTLESIM_WINDOW_HEIGHT) 
            request.theta = np.random.default_rng().uniform(low = 0.0, high = 360.0) 
            
            # New turtle name
            self.players_counter_ += 1
            request.name = "player_" + str(self.players_counter_)

            # Spawn new turtle asynchronously 
            future = client.call_async(request)
            future.add_done_callback(partial(self.spawn_player_done_, request=request))


    def spawn_player_done_(self, future, request):
        """Callback once a new player is added in the simulation."""

        # Check if turtle was spawned successfully.
        # Print success/failure message & add new turtle to list if spawning successfull.
        try:
            response = future.result()
        except Exception as e:
            node.get_logger().error("Could not spawn new turtle.")
        else:
            # New spawned turtle
            new_turtle = Turtle()
            
            new_turtle.name = response.name
            new_turtle.x = request.x
            new_turtle.y = request.y
            new_turtle.theta = request.theta

            # Update list of turtle players
            self.players_.append(new_turtle)

            # Print success message
            self.get_logger().info("New turtle spawned successfully.")


    def delete_player_(self, request_delete, response_delete):
        """Remove a player from the simulation."""

        # This callback uses the turtlesim /kill service
        # behind the scenes.
        #
        # Note that the /delete_turtle service differs from the /kill
        # service since it also needs to manage the list of turtle players.
        #
        # Variables in this method have a _delete or _kill suffix
        # to specify which service they are associated to.

        # Initialize response_delete
        response_delete.name = request_delete.name

        # Client for communicating with the /kill service.
        client_kill = self.create_client(Kill, "/kill")

        # Request for communicating with the /kill service.
        request_kill = Kill.Request()
        request_kill.name = request_delete.name

        # Call /kill service asynchronously
        future = client_kill.call_async(request_kill)
        future.add_done_callback(partial(self.kill_done_, request_kill=request_kill))

        # Return response_delete
        # Note: a response from service /delete_turtle means service /kill was called.
        #       The turtle might not have been removed from the list yet.
        return response_delete


    def kill_done_(self, future, request_kill):
        """Callback once a player has been removed from the simulation."""

        try:

            # Check if call to service /kill was successfull.
            # (successfull call = call which did not crash,
            # /kill can deal with non-existing turtle names)
            response_kill = future.result()

            # Remove tagged turtle from node internal list (if the turtle is in the list).
            for t in self.players_:
                if t.name == request_kill.name:
                    self.players_.remove(t)
                    break

            # By default /kill prints an error message if the turtle does not exist
            # and prints nothing if the deletion was successfull.
            # So no additional log message here.

        except Exception as e:

            log_message = "Call to /kill service failed."
            self.get_logger().error(log_message)


def main(args=None):

    rclpy.init(args=args)
    node = PlayersSpawnerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":

    main()


