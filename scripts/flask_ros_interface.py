#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from ros_interface import ROSInterface
import numpy as np
from flask import Flask, render_template, jsonify, request

class FlaskApp:
    def __init__(self):
        # Initialize Flask app
        self.app = Flask(__name__)
        # Create an instance of ROSInterface
        self.ros_handle = ROSInterface()

        # Define routes
        @self.app.route('/')
        def index():
            return render_template('flask_ros_interface.html')

        @self.app.route('/get_map_data', methods=['GET']) # map_origin is grid map's origin frame
        def map_data():
            return self.get_map_data()

        @self.app.route('/get_global_frame', methods=['GET'])
        def global_frame():
            return self.get_global_frame_in_map_origin() # global frame is map frame in ROS
        
        @self.app.route('/get_robot_pose', methods=['GET'])
        def robot_pose():
            return self.get_robot_in_map_origin()
        
        @self.app.route('/get_object_poses', methods=['GET'])
        def object_poses():
            return self.get_object_markers()

        @self.app.route('/send_goal', methods=['POST'])
        def send_goal():
            self.goal = request.json
            #rospy.loginfo("Goal pose : %s", self.goal)
            rospy.loginfo("Send goal to ROS navigation")
            self.set_goal()
            return jsonify({'message': 'Goal pose sent to ROS'})

    def get_object_markers(self):
        if self.ros_handle.object_markers_msg is not None:
            object_markers = []
            for marker in self.ros_handle.object_markers_msg.markers:
                object_in_global_mat = self.ros_handle.tf_merROS.fromTranslationRotation(
                                    (marker.pose.position.x, marker.pose.position.y, marker.pose.position.z), 
                                    (marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z, marker.pose.orientation.w))
                self.ros_handle.object_in_map_origin_mat = np.dot(self.ros_handle.global_in_map_origin_mat, object_in_global_mat)
                object_data = {
                    'name': marker.ns,
                    'position': {
                        'x': self.ros_handle.object_in_map_origin_mat[0][3],
                        'y': self.ros_handle.object_in_map_origin_mat[1][3],
                        'z': self.ros_handle.object_in_map_origin_mat[2][3]
                    },
                }
                object_markers.append(object_data)
            return jsonify(object_markers)
        else:
            return jsonify([])
        
    def get_map_data(self):
        if self.ros_handle.map_msg is not None:
            roll_map_origin, pitch_map_origin, yaw_map_origin = self.ros_handle.tf_mation.euler_from_quaternion([
                                                                self.ros_handle.map_msg.info.origin.orientation.x, 
                                                                self.ros_handle.map_msg.info.origin.orientation.y, 
                                                                self.ros_handle.map_msg.info.origin.orientation.z, 
                                                                self.ros_handle.map_msg.info.origin.orientation.w])
            return jsonify({
                'map_data': self.ros_handle.map_msg.data,
                'map_info': {
                    'width': self.ros_handle.map_msg.info.width,
                    'height': self.ros_handle.map_msg.info.height,
                    'resolution': self.ros_handle.map_msg.info.resolution,
                    'origin': {
                        'position': {
                            'x': self.ros_handle.map_msg.info.origin.position.x,
                            'y': self.ros_handle.map_msg.info.origin.position.y,
                            'z': self.ros_handle.map_msg.info.origin.position.z
                        },
                        'orientation': {
                            'roll': roll_map_origin,
                            'pitch': pitch_map_origin,
                            'yaw': yaw_map_origin
                        }
                    }
                }
            })
        else:
            rospy.logerr("Do not receive map topic")
            return jsonify({})

    def get_robot_in_map_origin(self):
        self.ros_handle.get_robot_to_map_origin_mat()
        roll_robot, pitch_robot, yaw_robot = self.ros_handle.tf_mation.euler_from_matrix(self.ros_handle.robot_in_map_origin_mat[:3,:3])
        #rospy.loginfo("Robot in map origin position x: %s, R matrix: %s", self.ros_handle.robot_in_map_origin_mat[0][3], self.ros_handle.robot_in_map_origin_mat[:3,:3])
        return jsonify({
            'position': {
                'x': self.ros_handle.robot_in_map_origin_mat[0][3],
                'y': self.ros_handle.robot_in_map_origin_mat[1][3],
                'z': self.ros_handle.robot_in_map_origin_mat[2][3]
            },
            'orientation': {
                'roll': roll_robot,
                'pitch': pitch_robot,
                'yaw': yaw_robot
            }
        })

    def get_global_frame_in_map_origin(self):
        self.ros_handle.get_global_to_map_origin_mat()
        roll_global, pitch_global, yaw_global = self.ros_handle.tf_mation.euler_from_matrix(self.ros_handle.global_in_map_origin_mat[:3,:3])   
        #rospy.loginfo("Global in map origin position x : %s, R matrix: %s", self.ros_handle.global_in_map_origin_mat[0][3], self.ros_handle.global_in_map_origin_mat[:3,:3])
        return jsonify({
            'position': {
                'x': self.ros_handle.global_in_map_origin_mat[0][3],
                'y': self.ros_handle.global_in_map_origin_mat[1][3],
                'z': self.ros_handle.global_in_map_origin_mat[2][3]
            },
            'orientation': {
                'roll': roll_global,
                'pitch': pitch_global,
                'yaw': yaw_global
            }
        })

    def set_goal(self):
        self.ros_handle.goal_in_global_mat = np.dot(self.ros_handle.map_origin_in_global_mat,
                                                        self.ros_handle.tf_merROS.fromTranslationRotation((self.goal['position']['x'],self.goal['position']['y'],0.0),
                                                                                                                self.ros_handle.tf_mation.quaternion_from_euler(0.0,0.0,self.goal['orientation']['yaw'])))
        #rospy.loginfo("Goal in Global frame : \n%s", self.ros_handle.goal_in_global_mat)
        self.ros_handle.publish_goal()

    def run(self):
        # Run the Flask app
        self.app.run(host='0.0.0.0', port=7080, debug=True)

if __name__ == '__main__':
    rospy.init_node('flask_robot')
    app = FlaskApp()
    app.run()
