import rospy
from uwb_wsn_simulation.msg import uwb_wsn_data
from uwb_wsn_localization.srv import estimation_result, estimation_resultResponse
from uwb_wsn_localization.msg import uwb_wsn_slam_data
from geometry_msgs.msg import Point
from uwb_sensor_utility import get_sensor_pos
import numpy as np


global sensor_position
sensor_position = []
global localized
localized = set()  # sensor id which is localized
global anchor_id
anchor_id = set()

global ekf_result
ekf_result = []
global ekf_localized
ekf_localized = []


def ekf_data_update(data: uwb_wsn_slam_data):
    rospy.loginfo_once('ekf_data_update start to work')
    global ekf_result
    ekf_result = data.pose_vector
    global ekf_localized
    ekf_localized = data.localized_ekf


def maximum_likelihood_estimation(position, distance):
    '''
    position: a list, each element is the position of known sensor
    distance: a list, each element is the distance from known sensor to unknown sensor
    '''
    xn = position[-1][0]
    yn = position[-1][1]
    dn = distance[-1]

    for i in range(len(position) - 1):
        xi = position[i][0]
        yi = position[i][1]
        di = distance[i]
        if i == 0:
            A = np.array([2*(xi - xn), 2*(yi - yn)])
            B = np.array([xi**2 - xn**2 + yi**2 - yn**2 + dn**2 - di**2])
        else:
            A = np.vstack((A, np.array([2*(xi - xn), 2*(yi - yn)])))
            B = np.vstack(
                (B, np.array([xi**2 - xn**2 + yi**2 - yn**2 + dn**2 - di**2])))

    temp = np.linalg.inv(np.dot(A.T, A))
    X = np.dot(np.dot(temp, A.T), B)

    p = Point()
    p.x = X[0][0]
    p.y = X[1][0]
    return p


def uwb_wsn_data_update(data: uwb_wsn_data):
    global uwb_wsn_data_
    uwb_wsn_data_ = data.uwb_wsn_data

    # init a list of sensor position from maximum likelihood estimation
    global sensor_position
    if len(sensor_position) == 0:
        # the number of uwb_wsn_data_ equals the number of uwb_sensor
        sensor_position = [[]] * len(uwb_wsn_data_)

    # update sensor_position & localized & anchor_id with anchor position
    global anchor_pos
    global localized
    for each in uwb_wsn_data_:
        if each.is_anchor:
            p = Point()
            # robot's node_id is 0, other sensor id start from 1
            # anchor_pos[0] means node_id=1
            p.x = anchor_pos[each.node_id - 1][0]
            p.y = anchor_pos[each.node_id - 1][1]
            sensor_position[each.node_id] = [p]
            localized.add(each.node_id)
            anchor_id.add(each.node_id)


def request_handler(req):
    rospy.loginfo('clear old data')
    global sensor_position
    for i in range(len(sensor_position)):
        sensor_position[i] = []
    global localized
    localized = set()
    global anchor_id
    anchor_id = set()
    rospy.sleep(0.5)  # wait for uwb_wsn_data_update

    # add ekf data
    global ekf_localized
    for sensor in ekf_localized:
        if sensor != 0:
            x = ekf_result[2*sensor+1]
            y = ekf_result[2*sensor+2]
        else:
            x = ekf_result[sensor]
            y = ekf_result[sensor+1]
        
        p = Point()
        p.x = x
        p.y = y
        sensor_position[sensor] = [p]
        localized.add(sensor)

    rospy.loginfo('estimation start...')
    count = 0
    while len(localized) < len(uwb_wsn_data_) and count < 500:
        count += 1

        for each in uwb_wsn_data_:
            if each.node_id not in localized:
                # this sensor is not localized
                position = []
                distance = []
                for index, id in enumerate(each.destination_id):
                    if id in localized:
                        # put localized sensor in communication range in list
                        position.append(
                            [sensor_position[id][0].x, sensor_position[id][0].y])
                        distance.append(each.distance[index])
                if len(position) < 3 or len(distance) < 3:
                    # unable to use maximum likelihood estimation
                    continue
                sensor_position[each.node_id] = [
                    maximum_likelihood_estimation(position, distance)]
                localized.add(each.node_id)

    rospy.loginfo('estimation finished in %d steps.', count)

    # process not localized sensor
    for i in range(len(sensor_position)):
        if len(sensor_position[i]) == 0:
            rospy.loginfo('sensor %d is unable to localize, set by (0,0).', i)
            p = Point()
            p.x = p.y = 0
            sensor_position[i] = [p]

    res = estimation_resultResponse()
    for each in sensor_position:
        res.result.append(each[0])
    res.anchor_id = anchor_id
    res.localized = list(localized)

    return res


if __name__ == '__main__':
    rospy.init_node('maximum_likelihood_estimation', anonymous=True)

    global anchor_pos
    anchor_pos = get_sensor_pos('anchor', 3)

    rospy.Subscriber('uwb_wsn_data_topic', uwb_wsn_data, uwb_wsn_data_update)
    rospy.Subscriber('ekf_localization_data', uwb_wsn_slam_data, ekf_data_update)

    rospy.Service('maximum_likelihood_estimation_server',
                  estimation_result, request_handler)
    rospy.loginfo('Service: maximum_likelihood_estimation_server ready.')

    rospy.spin()
