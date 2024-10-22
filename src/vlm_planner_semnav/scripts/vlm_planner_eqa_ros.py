#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import hydra_msgs.msg
import spark_dsg as dsg
from sensor_msgs.msg import Image
from semantic_inference_msgs.msg import FeatureImage

# Copyright 2022, Massachusetts Institute of Technology.
# All Rights Reserved
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#  1. Redistributions of source code must retain the above copyright notice,
#     this list of conditions and the following disclaimer.
#
#  2. Redistributions in binary form must reproduce the above copyright notice,
#     this list of conditions and the following disclaimer in the documentation
#     and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Research was sponsored by the United States Air Force Research Laboratory and
# the United States Air Force Artificial Intelligence Accelerator and was
# accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views
# and conclusions contained in this document are those of the authors and should
# not be interpreted as representing the official policies, either expressed or
# implied, of the United States Air Force or the U.S. Government. The U.S.
# Government is authorized to reproduce and distribute reprints for Government
# purposes notwithstanding any copyright notation herein.
#

"""Conversion from spark_dsg to networkx."""
import importlib
import logging
import json
import numpy as np

def _get_networkx():
    networkx = None
    try:
        networkx = importlib.import_module("networkx")
    except ImportError:
        logging.warning("networkx not found. conversion disabled")

    return networkx


def _convert_attr(attrs):
    valid_fields = [x for x in dir(attrs) if x[0] != "_"]
    return {x: getattr(attrs, x) for x in valid_fields}


def _fill_from_layer(G_out, layer):
    for node in layer.nodes:
        G_out.add_node(node.id.value, **_convert_attr(node.attributes))

    for edge in layer.edges:
        G_out.add_edge(edge.source, edge.target, **_convert_attr(edge.info))


def graph_to_networkx(G_in, include_dynamic=True):
    """Convert the DSG to a networkx representation."""
    nx = _get_networkx()
    if nx is None:
        return None

    G_out = nx.Graph()
    for layer in G_in.layers:
        _fill_from_layer(G_out, layer)

    for edge in G_in.interlayer_edges:
        G_out.add_edge(edge.source, edge.target, **_convert_attr(edge.info))

    if not include_dynamic:
        return G_out

    for layer in G_in.dynamic_layers:
        _fill_from_layer(G_out, layer)

    for edge in G_in.dynamic_interlayer_edges:
        G_out.add_edge(edge.source, edge.target, **_convert_attr(edge.info))

    return G_out


def layer_to_networkx(G_in):
    """Convert the DSG to a networkx representation."""
    nx = _get_networkx()
    if nx is None:
        return None

    G_out = nx.Graph()
    _fill_from_layer(G_out, G_in)
    return G_out

# BEGIN TOOLS DEVELOPED FOR SEMNAV

# Checkes if a value within a networkx graph is json-serializable
# TODO(blake.buchanan): Currently everything not directly json serializable
# is being ignored. Don't filter ndarrays, but convert them to lists and then
# output them via json.dump().
def is_json_serializable(key, value):
    try:
        json.dumps(value)
        return True
    except (TypeError, OverflowError):
        return False

# Filters a networkx graph to contain nodes only with attributes
# which can be serialized when calling json.dump().
def filter_serializable_graph(graph):
    import networkx as nx
    filtered_graph = nx.Graph()  # or nx.DiGraph() depending on your graph type
    keys_to_filter = ["deformation_connections", "mesh_vertex_labels", "pcl_mesh_connections",
                      "num_frontier_voxels", "NO_SEMANTIC_LABEL", "need_cleanup", "ellipse_centroid",
                      "ellipse_matrix_compress", "ellipse_matrix_expand", "pcl_boundary_connections"]

    # Add nodes with serializable data
    for node, data in graph.nodes(data=True):
        serializable_data = {k: (v.tolist() if isinstance(v, np.ndarray) else v)
                                    for k, v in data.items()
                                    if is_json_serializable(k, v.tolist() if isinstance(v, np.ndarray) else v)
                                    and k not in keys_to_filter}
        filtered_graph.add_node(node, **serializable_data)

    # Add edges with serializable data
    for u, v, data in graph.edges(data=True):
        serializable_data = {k: (v.tolist() if isinstance(v, np.ndarray) else v)
                                    for k, v in data.items()
                                    if is_json_serializable(k, v.tolist() if isinstance(v, np.ndarray) else v)
                                    and k not in keys_to_filter}
        filtered_graph.add_edge(u, v, **serializable_data)

    return filtered_graph

class HydraSceneGraphSubscriber:
    """Class for receiving a scene graph from the Hydra package in python."""

    def __init__(self, topic, callback):
        """Construct a subscriber."""
        self._callback = callback

        # Not sure what the message types will be here
        self._sub = rospy.Subscriber(
            topic, hydra_msgs.msg.DsgUpdate, self._callback
        )
        rospy.loginfo(f"Created subscriber for topic {topic}")

class VLMPlannerNode:
    """Node to handle listening for a scene graph, adding nodes to a NetworkX graph, and outputting the graph as a JSON file."""

    def __init__(self, with_mesh=True):
        """Start a listener node."""
        self._scene_graph_sub = HydraSceneGraphSubscriber("/hydra_ros_node/backend/dsg", self.hydraSceneGraphCallback)
        self._semantic_image_sub = rospy.Subscriber("/camera/semantic/image_raw_test", FeatureImage, self.semanticImageCallback)
        # TODO(blake) Need a way to get the state of the Stretch. vlm_planner.get_next_action() will need to consider Stretch
        self._semantic_image_pub = rospy.Publisher("/camera/semantic/image_raw", Image, queue_size=10)
    
    
    def hydraSceneGraphCallback(self, msg):
        rospy.loginfo("Got a dynamic scene graph message...")
        # This should get the binary data and convert it to type DynamicSceneGraph
        scene_graph = dsg.DynamicSceneGraph.from_binary(msg.layer_contents)

        # Then use networkx.py to conver to networkx format
        netx_scene_graph = graph_to_networkx(scene_graph)

        # Then convert to JSON using native networkx tools
        # Convert the graph to a dictionary in JSON format
        import networkx as nx
        import json

        # All of the data in graph_data is not JSON-serializable,
        # So we parse graph_data before writing to json
        filtered_graph = filter_serializable_graph(netx_scene_graph)

        graph_data = nx.node_link_data(filtered_graph)

        # Write the JSON data to a file
        with open("graph.json", "w") as f:
            json.dump(graph_data, f, indent=4)

    def semanticImageCallback(self, semantic_msg):
        rospy.loginfo("Got a semantic image...")
        output_msg = semantic_msg.image
        self._semantic_image_pub.publish(output_msg)
        rospy.loginfo("Publishing converged image to Hydra...")

    def spin(self):
        """Wait until rospy is shutdown."""
        rospy.spin()


def main():
    """Start ROS and the node."""
    rospy.init_node("hydra_scene_graph_listener")

    node = VLMPlannerNode()

    node.spin()


if __name__ == "__main__":
    main()
