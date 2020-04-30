#! /usr/bin/env python
import rospy
import tf
from gazebo_msgs.srv import GetModelState

def main():
    model_coordinates = rospy.ServiceProxy( '/gazebo/get_model_state', GetModelState)
    object_coordinates = model_coordinates("unit_box", "")
    z_position = object_coordinates.pose.position.z
    y_position = object_coordinates.pose.position.y
    x_position = object_coordinates.pose.position.x
    print('x position is ')
    print(x_position)
    print('\n')
    print('y position is ')
    print(y_position)
    print('\n')
    print(object_coordinates.pose)
    print(model_coordinates)

if __name__ == "__main__":
    main()
