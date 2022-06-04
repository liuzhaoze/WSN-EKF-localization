import rospy
from visualization_msgs.msg import Marker, MarkerArray
from uwb_wsn_simulation.msg import uwb_wsn_data
from uwb_sensor_utility import get_sensor_pos
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point


def robot_pose_update(data: Odometry):
    global robot_pos
    robot_pos = [data.pose.pose.position.x,
                 data.pose.pose.position.y, data.pose.pose.position.z]


def plot_wsn_state(uwb_wsn_data: uwb_wsn_data):
    global robot_pos
    global anchor_pos
    global unknown_pos

    sensor_pose = [robot_pos] + anchor_pos + unknown_pos
    marker_array = MarkerArray()

    # plot sensor point
    sensor_points = Marker()
    sensor_points.header.frame_id = 'map'
    sensor_points.header.stamp = rospy.Time.now()
    sensor_points.ns = 'sensor_points'
    sensor_points.action = Marker.MODIFY
    sensor_points.pose.orientation.w = 1.0
    sensor_points.id = 0
    sensor_points.type = Marker.POINTS
    sensor_points.scale.x = 0.08
    sensor_points.scale.y = 0.08
    sensor_points.color.g = 0.4  # green
    sensor_points.color.a = 1.0
    for pos in sensor_pose:
        p = Point()
        p.x = pos[0]
        p.y = pos[1]
        sensor_points.points.append(p)
    marker_array.markers.append(sensor_points)

    # plot communication line
    count = 0
    for node_data in uwb_wsn_data.uwb_wsn_data:
        for each in node_data.destination_id:
            communication_line = Marker()
            communication_line.header.frame_id = 'map'
            communication_line.header.stamp = rospy.Time.now()
            communication_line.ns = 'communication_line'
            communication_line.action = Marker.MODIFY
            communication_line.pose.orientation.w = 1.0
            communication_line.id = count
            count += 1
            communication_line.type = Marker.LINE_STRIP
            communication_line.scale.x = 0.02
            communication_line.scale.y = 0.02
            communication_line.color.g = 0.7  # green
            communication_line.color.a = 1.0
            communication_line.lifetime = rospy.Duration(0.5)

            p1 = Point()
            p1.x = sensor_pose[node_data.node_id][0]
            p1.y = sensor_pose[node_data.node_id][1]
            p2 = Point()
            p2.x = sensor_pose[each][0]
            p2.y = sensor_pose[each][1]

            communication_line.points = [p1, p2]

            marker_array.markers.append(communication_line)

    pub.publish(marker_array)


if __name__ == '__main__':
    rospy.init_node('wsn_state_plotter', anonymous=True)

    global pub
    pub = rospy.Publisher('visualization_marker_array_simulation',
                          MarkerArray, queue_size=10)
    global anchor_pos
    anchor_pos = get_sensor_pos('anchor', 3)
    global unknown_pos
    unknown_pos = get_sensor_pos('unknown', 100)

    rospy.Subscriber('uwb_wsn_data_topic', uwb_wsn_data, plot_wsn_state)
    rospy.Subscriber('/ground_truth/state', Odometry, robot_pose_update)

    rospy.spin()
