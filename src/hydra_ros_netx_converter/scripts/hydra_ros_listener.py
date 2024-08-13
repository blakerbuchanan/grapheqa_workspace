#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import hydra_msgs.msg
import spark_dsg as dsg

class HydraSceneGraphSubscriber:
    """Class for receiving a scene graph in python."""

    def __init__(self, topic, callback):
        """Construct a DSG Receiver."""
        self._callback = callback
        topic = "/hydra_ros_node/frontend/dsg"

        # Not sure what the message types will be here
        self._sub = rospy.Subscriber(
            topic, hydra_msgs.msg.DsgUpdate, self.handleUpdate
        )
        rospy.loginfo(f"Created subscriber for topic {topic}")

    def handleUpdate(self, msg):
        rospy.loginfo("Got a dynamic scene graph message...")
        scene_graph = spark_dsg.DynamicSceneGraph.from_binary(msg.layer_contents)

class HydraSceneGraphListenerNode:
    """Node to handle listening for a scene graph, adding nodes to a NetworkX graph, and outputting the graph as a JSON file."""

    def __init__(self, with_mesh=True):
        """Start a listener node."""
        self._sub = HydraSceneGraphSubscriber("dsg_in", self.hydraSceneGraphCallback)

    def hydraSceneGraphCallback(self, header, G):
        rospy.loginfo(
            f"Received graph with {G.num_nodes()} nodes @ {header.stamp.to_nsec()} [ns]"
        )

    def spin(self):
        """Wait until rospy is shutdown."""
        rospy.spin()


def main():
    """Start ROS and the node."""
    rospy.init_node("hydra_scene_graph_listener")

    node = HydraSceneGraphListenerNode()

    node.spin()


if __name__ == "__main__":
    main()
