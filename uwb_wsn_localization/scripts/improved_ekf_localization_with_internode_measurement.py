import rospy
from uwb_wsn_localization.srv import estimation_result
import numpy as np
import rostopic
from nav_msgs.msg import Odometry
from uwb_wsn_localization.msg import uwb_wsn_slam_data
from geometry_msgs.msg import PoseWithCovariance
from uwb_wsn_simulation.msg import uwb_wsn_data


def normalize_angle(phi):
    '''
    Normalize phi to be between -pi and pi
    '''
    while phi > np.pi:
        phi -= 2 * np.pi
    while phi < -np.pi:
        phi += 2 * np.pi
    return phi


def initialization():
    rospy.wait_for_service('maximum_likelihood_estimation_server')
    try:
        maximum_likelihood_estimation = rospy.ServiceProxy(
            'maximum_likelihood_estimation_server', estimation_result)
        response = maximum_likelihood_estimation()
        init_pos = response.result
        anchor_id = response.anchor_id
        localized_list = response.localized
    except rospy.ServiceException as e:
        rospy.logerr(str(e))

    global mu
    mu = np.zeros(2 * len(init_pos))
    for index, point in enumerate(init_pos):
        mu[2*index:2*index+2] = np.array([point.x, point.y])
    mu = np.insert(mu, 2, 0)  # insert robot pose: theta

    global sigma
    sigma = 10 * np.identity(mu.shape[0])
    for id in anchor_id:
        # anchor has lower variance
        sigma[id*2+1, id*2+1] = 0.1
        sigma[id*2+2, id*2+2] = 0.1

    global localized_ekf
    localized_ekf = set(localized_list)


def publish_data(mu, sigma):
    slam_result = uwb_wsn_slam_data()
    slam_result.pose_vector = list(mu)

    # generate robot position
    p = PoseWithCovariance()
    p.pose.position.x = mu[0]
    p.pose.position.y = mu[1]
    # [cov(X, X), cov(Y, Y), cov(X, Y)]
    cov = [sigma[0, 0], sigma[1, 1], sigma[0, 1]]
    p.covariance = cov + [0] * (36 - len(cov))
    slam_result.result.append(p)

    # generate sensor position
    sensor_num = int((mu.shape[0] - 3) / 2)
    for i in range(sensor_num):
        p = PoseWithCovariance()
        p.pose.position.x = mu[3 + i * 2]
        p.pose.position.y = mu[3 + i * 2 + 1]
        # [cov(X, X), cov(Y, Y), cov(X, Y)]
        cov = [sigma[3 + i * 2, 3 + i * 2],
               sigma[3 + i * 2 + 1, 3 + i * 2 + 1],
               sigma[3 + i * 2, 3 + i * 2 + 1]]
        p.covariance = cov + [0] * (36 - len(cov))
        slam_result.result.append(p)

    global localized_ekf
    slam_result.localized_ekf = list(localized_ekf)

    pub.publish(slam_result)


def prediction_step(Odometry: Odometry):
    '''
    mu: 3+2N x 1 vector
    sigma: 3+2N x 3+2N covariance matrix
    '''
    global mu
    global sigma
    global odom_hz

    x = mu[0]
    y = mu[1]
    theta = mu[2]

    delta_d = Odometry.twist.twist.linear.x / odom_hz
    delta_theta = Odometry.twist.twist.angular.z / odom_hz

    noise = 0.1**2
    d_noise = delta_d**2
    theta_noise = delta_theta**2

    # predict position
    mu[0] = x + delta_d * np.cos(theta + delta_theta)
    mu[1] = y + delta_d * np.sin(theta + delta_theta)
    mu[2] = normalize_angle(theta + delta_theta)

    # predict covariance matrix
    # A is the Jacobian matrix of process function f() with respect to state x
    A = np.identity(mu.shape[0])
    A[0, 2] = -delta_d * np.sin(theta + delta_theta)
    A[1, 2] = delta_d * np.cos(theta + delta_theta)
    # Q is the covariance matrix of process noise
    Q = np.zeros(sigma.shape)
    Q[0][0] = Q[1][1] = noise + d_noise
    Q[2][2] = noise + theta_noise
    # calculate sigma
    sigma = np.dot(np.dot(A, sigma), A.T) + Q

    np.set_printoptions(formatter={'float': '{: 0.3f}'.format})
    # rospy.loginfo(mu)

    return mu, sigma


REQUEST_LIMIT = 3
request_count = {}


def correction_step(uwb_wsn_data: uwb_wsn_data):
    '''
    mu: 3+2N x 1 vector
    sigma: 3+2N x 3+2N covariance matrix
    '''
    global mu
    global sigma
    global localized_ekf

    uwb_wsn_data_ = uwb_wsn_data.uwb_wsn_data
    for each in uwb_wsn_data_:
        if each.node_id == 0:
            robot_sensor_data = each

    # initialization of sensor data
    sensor_index = list(robot_sensor_data.destination_id)
    sensor_distance = np.array(robot_sensor_data.distance)

    # process the sensor data that isn't localized
    global request_count
    for index, sensor in enumerate(sensor_index):
        if sensor not in localized_ekf:
            if str(sensor) not in request_count:
                request_count[str(sensor)] = 0
            elif request_count[str(sensor)] >= REQUEST_LIMIT:
                continue

            rospy.loginfo('find unknown sensor: %d', sensor)
            # try to estimate again if robot is around
            rospy.wait_for_service('maximum_likelihood_estimation_server')
            try:
                maximum_likelihood_estimation = rospy.ServiceProxy(
                    'maximum_likelihood_estimation_server', estimation_result)
                response = maximum_likelihood_estimation()
                request_count[str(sensor)] += 1
                init_pos = response.result
                localized_list = response.localized
            except rospy.ServiceException as e:
                rospy.logerr(str(e))

            if sensor in localized_list:
                # estimation success, update mu with new value
                rospy.loginfo('sensor: %d estimation success, request: %d',
                              sensor, request_count[str(sensor)])
                if sensor != 0:
                    mu[2*sensor+1] = init_pos[sensor].x
                    mu[2*sensor+2] = init_pos[sensor].y
                    localized_ekf.add(sensor)
                else:
                    mu[sensor] = init_pos[sensor].x
                    mu[sensor+1] = init_pos[sensor].y
                    localized_ekf.add(sensor)
            else:
                # estimation fail, remove sensor data incase error
                rospy.loginfo('sensor: %d estimation fail, request: %d',
                              sensor, request_count[str(sensor)])
                del sensor_index[index]
                sensor_distance = np.delete(sensor_distance, index, 0)

    # the number of sensor in communication range is m
    distance_num = len(sensor_index)

    # calculate kalman gain and predicted distance
    predicted_distance = np.zeros(distance_num)
    # H is the Jacobian matrix of measure function h() with respect to status x
    # H: m x 3+2N (m will increase due to internode measurement)
    H = np.zeros((distance_num, sigma.shape[0]))

    for i, sensor_id in enumerate(sensor_index):
        Hi = np.zeros(H.shape[1])
        robot_x = mu[0]
        robot_y = mu[1]
        # sensor id start from 1
        sensor_x = mu[sensor_id * 2 + 1]
        sensor_y = mu[sensor_id * 2 + 2]
        # compute the expected distance for each landmark
        distance_exp = np.sqrt((robot_x - sensor_x) **
                               2 + (robot_y - sensor_y)**2)
        predicted_distance[i] = distance_exp

        Hi[0] = (robot_x - sensor_x) / distance_exp
        Hi[1] = (robot_y - sensor_y) / distance_exp
        Hi[sensor_id * 2 + 1] = (sensor_x - robot_x) / distance_exp
        Hi[sensor_id * 2 + 2] = (sensor_y - robot_y) / distance_exp
        H[i, :] = Hi

    # add internode measurement
    for each in uwb_wsn_data_:
        if each.node_id in localized_ekf and each.node_id != 0:
            # exclude robot which id is 0
            for index, neighbor in enumerate(each.destination_id):
                if neighbor in localized_ekf and neighbor != 0:
                    Hi = np.zeros(H.shape[1])
                    sensor1_x = mu[each.node_id * 2 + 1]
                    sensor1_y = mu[each.node_id * 2 + 2]
                    sensor2_x = mu[neighbor * 2 + 1]
                    sensor2_y = mu[neighbor * 2 + 2]

                    distance_exp = np.sqrt(
                        (sensor1_x - sensor2_x)**2 + (sensor1_y - sensor2_y)**2)
                    predicted_distance = np.append(
                        predicted_distance, distance_exp)
                    sensor_distance = np.append(
                        sensor_distance, each.distance[index])

                    Hi[each.node_id * 2 +
                        1] = (sensor1_x - sensor2_x) / distance_exp
                    Hi[each.node_id * 2 +
                        2] = (sensor1_y - sensor2_y) / distance_exp
                    Hi[neighbor * 2 + 1] = (sensor2_x -
                                            sensor1_x) / distance_exp
                    Hi[neighbor * 2 + 2] = (sensor2_y -
                                            sensor1_y) / distance_exp
                    H = np.vstack((H, Hi))

    # R is the covariance matrix of measurement noise
    R = 0.5 * np.identity(len(sensor_distance))
    # K is kalman gain
    K_help = np.linalg.inv(np.dot(np.dot(H, sigma), H.T) + R)
    K = np.dot(np.dot(sigma, H.T), K_help)

    # correct status mu
    mu = mu + np.dot(K, sensor_distance - predicted_distance)
    mu[2] = normalize_angle(mu[2])

    # correct covariance matrix sigma
    sigma = np.dot(np.identity(sigma.shape[0]) - np.dot(K, H), sigma)

    return mu, sigma


def subscribe_odom_data(Odometry):
    [mu, sigma] = prediction_step(Odometry)
    publish_data(mu, sigma)


def subscribe_uwb_wsn_data(uwb_data):
    [mu, sigma] = correction_step(uwb_data)


if __name__ == '__main__':
    rospy.init_node('efk_localization', anonymous=True)
    initialization()

    # get odom hz
    global odom_hz
    odom_topic_name = "odom"
    r = rostopic.ROSTopicHz(-1)
    s = rospy.Subscriber('/' + odom_topic_name, Odometry,
                         r.callback_hz, callback_args="/odom")
    rospy.sleep(1)
    odom_hz = int(r.get_hz('/' + odom_topic_name)[0])
    s.unregister()

    global pub
    pub = rospy.Publisher("ekf_localization_data",
                          uwb_wsn_slam_data, queue_size=10)
    r = rospy.Rate(1)

    rospy.Subscriber("odom", Odometry, subscribe_odom_data)
    rospy.Subscriber("uwb_wsn_data_topic", uwb_wsn_data,
                     subscribe_uwb_wsn_data)

    rospy.spin()
