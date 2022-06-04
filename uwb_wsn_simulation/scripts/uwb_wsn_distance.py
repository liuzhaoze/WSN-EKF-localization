import rospy
from nav_msgs.msg import Odometry
from uwb_sensor_utility import get_sensor_pos
from uwb_wsn_simulation.msg import uwb_node_data, uwb_wsn_data
import numpy as np


def ground_truth_callback(data: Odometry):
    robot_pos = [data.pose.pose.position.x,
                 data.pose.pose.position.y, data.pose.pose.position.z]
    global_data = uwb_wsn_data()

    sensor_poses = [robot_pos] + anchor_pos + unknown_pos
    for index, position in enumerate(sensor_poses):
        local_data = uwb_node_data()
        # 0 is the sensor on robot, anchor is after robot, unknown is after anchor
        local_data.node_id = index
        if 0 < local_data.node_id <= len(anchor_pos):
            local_data.is_anchor = True
        else:
            local_data.is_anchor = False

        for i, p in enumerate(sensor_poses):
            if local_data.node_id != i:
                # is not itself
                p1 = np.array(position)
                p2 = np.array(p)
                distance = np.sqrt(np.sum((p1 - p2)**2, axis=0))
                if distance <= 3.0:
                    local_data.destination_id.append(i)
                    local_data.distance.append(
                        distance + np.random.normal(0, distance*0.015, 1))
        global_data.uwb_wsn_data.append(local_data)

    pub.publish(global_data)


if __name__ == '__main__':
    rospy.init_node('uwb_wsn_distance', anonymous=True)
    global pub
    pub = rospy.Publisher('uwb_wsn_data_topic', uwb_wsn_data, queue_size=10)
    global anchor_pos
    anchor_pos = get_sensor_pos('anchor', 3)
    global unknown_pos
    unknown_pos = get_sensor_pos('unknown', 100)

    rospy.sleep(0.5)
    rospy.Subscriber('/ground_truth/state', Odometry, ground_truth_callback)

    rospy.spin()
