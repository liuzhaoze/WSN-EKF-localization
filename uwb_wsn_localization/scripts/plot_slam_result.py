import rospy
import numpy as np
from uwb_wsn_localization.msg import uwb_wsn_slam_data
from geometry_msgs.msg import Point, PoseWithCovariance
from visualization_msgs.msg import Marker, MarkerArray
from scipy.stats.distributions import chi2


position_vector = []


def ellipse_points(position: PoseWithCovariance, alpha, scale=1.0):
    x = position.pose.position.x
    y = position.pose.position.y
    XX = position.covariance[0]
    YY = position.covariance[1]
    XY = position.covariance[2]

    # Calculate unscaled half axes
    temp = np.array([0.5*(XX+YY+np.sqrt((XX-YY)**2+4*(XY**2))),
                     0.5*(XX+YY-np.sqrt((XX-YY)**2+4*(XY**2)))],
                    dtype=np.complex)
    temp = np.sqrt(temp)
    # Remove imaginary parts in case of neg. definite Cov
    a = temp[0].real
    b = temp[1].real
    # Scaling in order to reflect specified probability
    a = a * np.sqrt(chi2.ppf(alpha, 2))
    b = b * np.sqrt(chi2.ppf(alpha, 2))
    # adjust the scale
    a *= scale
    b *= scale
    # Look where the greater half axis belongs to
    if XX < YY:
        swap = a
        a = b
        b = swap
    # Calculate inclination (numerically stable)
    if XX != YY:
        angle = 0.5 * np.arctan(2*XY/(XX-YY))
    elif XY == 0:
        angle = 0     # angle doesn't matter
    elif XY > 0:
        angle = np.pi / 4
    elif XY < 0:
        angle = -np.pi / 4

    # generate points
    NPOINTS = 100
    index = np.linspace(0, 2*np.pi, NPOINTS)
    p = np.zeros((2, NPOINTS))
    p[0, :] = a * np.cos(index)
    p[1, :] = b * np.sin(index)
    R = np.array([[np.cos(angle), -np.sin(angle)],
                  [np.sin(angle), np.cos(angle)]])
    T = np.dot(np.array([[x], [y]]), np.ones((1, index.shape[0])))
    p = np.dot(R, p) + T

    points = []
    for i in range(p.shape[1]):
        geo_p = Point()
        geo_p.x = p[0, i]
        geo_p.y = p[1, i]
        points.append(geo_p)

    return points


def plot_markers(uwb_slam_data: uwb_wsn_slam_data):
    global position_vector
    position_vector = uwb_slam_data.pose_vector  # update position_vector

    marker_array = MarkerArray()

    location_points = Marker()
    location_points.header.frame_id = "map"
    location_points.header.stamp = rospy.Time.now()
    location_points.ns = "location_points"
    location_points.action = Marker.MODIFY
    location_points.pose.orientation.w = 1.0
    location_points.id = 0
    location_points.type = Marker.POINTS
    location_points.scale.x = 0.1
    location_points.scale.y = 0.1
    location_points.color.b = 1.0  # blue
    location_points.color.a = 1.0
    location_points.lifetime = rospy.Duration(0.5)

    for index, pos in enumerate(uwb_slam_data.result):
        if index not in uwb_slam_data.localized_ekf:
            continue
        location_points.points.append(pos.pose.position)

        prob_ellipse = Marker()
        prob_ellipse.header.frame_id = "map"
        prob_ellipse.header.stamp = rospy.Time.now()
        prob_ellipse.ns = "prob_ellipse"
        prob_ellipse.action = Marker.MODIFY
        prob_ellipse.pose.orientation.w = 1.0
        prob_ellipse.id = index
        prob_ellipse.type = Marker.LINE_STRIP
        prob_ellipse.scale.x = 0.02
        prob_ellipse.scale.y = 0.02
        prob_ellipse.color.r = 1.0  # red
        prob_ellipse.color.a = 1.0
        prob_ellipse.points = ellipse_points(pos, alpha=0.6, scale=0.5)
        prob_ellipse.lifetime = rospy.Duration(0.5)

        marker_array.markers.append(prob_ellipse)

    marker_array.markers.append(location_points)
    pub.publish(marker_array)


if __name__ == '__main__':
    rospy.init_node('slam_result_plotter', anonymous=True)

    global pub
    pub = rospy.Publisher("visualization_marker_array_localization",
                          MarkerArray, queue_size=10)

    rospy.Subscriber("ekf_localization_data", uwb_wsn_slam_data, plot_markers)

    rospy.spin()
