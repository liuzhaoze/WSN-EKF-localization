import rospy
import tf
import tf2_ros


def get_sensor_pos(sensor_type: str, sensor_num: int):
    sensor_pos = []
    listener = tf.TransformListener()

    for i in range(sensor_num):
        try:
            sensor_name = 'uwb_' + sensor_type + '_' + str(i)
            rospy.loginfo('get position: ' + sensor_name)

            listener.waitForTransform(
                'map', sensor_name, rospy.Time(0), rospy.Duration(2.0))
            t = listener.getLatestCommonTime('map', sensor_name)
            (trans, rot) = listener.lookupTransform('map', sensor_name, t)
            sensor_pos.append(trans)
        except (tf2_ros.TransformException, tf2_ros.LookupException):
            break

    if len(sensor_pos) == 0:
        rospy.logerr("There is not any " + sensor_type + " sensor found.")
    else:
        rospy.logwarn("%d %s sensor(s) found.", len(sensor_pos), sensor_type)
        rospy.logwarn(str(sensor_pos))

    return sensor_pos
