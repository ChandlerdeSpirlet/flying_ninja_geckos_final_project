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

    dimensions = (1, 1)
    for x in range(0, 3):
        obj_coord = model_coordinates(str(x), "")
        x = obj_coord.pose.position.x
        y = obj_coord.pose.position.y
        position = (x - 0.5, y - 0.5)


if __name__ == "__main__":
    main()
