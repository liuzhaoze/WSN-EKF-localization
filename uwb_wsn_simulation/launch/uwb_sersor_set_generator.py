import random

############ config ############
# SENSOR_TYPE = 'anchor'
SENSOR_TYPE = 'unknown'
RANDOM_SENSOR = True
SPAWN_MODEL = True
SENSOR_NUM = 12
X_RANGE = [-4, 4]
Y_RANGE = [-4, 4]
################################


# generate position
x = []
y = []

if RANDOM_SENSOR:
    for i in range(SENSOR_NUM):
        x.append(random.uniform(X_RANGE[0], X_RANGE[1]))
        y.append(random.uniform(Y_RANGE[0], Y_RANGE[1]))
else:
    ######## modified if necessary ########
    x = [0, 0, 0.866]
    y = [0.5, -0.5, 0]
    #######################################
    if len(x) != SENSOR_NUM or len(y) != SENSOR_NUM:
        raise Exception('x/y not fit SENSOR_NUM:{}'.format(SENSOR_NUM))


# write file
file_name = 'uwb_' + SENSOR_TYPE + '_set.launch'
f = open(file_name, 'w')

f.write('''<launch>\n''')
f.write('''\t<arg name="uwb_{}_model" default="$(find uwb_wsn_simulation)/models/{}_sensor/model.sdf" />\n\n'''.format(SENSOR_TYPE, SENSOR_TYPE))

for i in range(SENSOR_NUM):
    uwb_name = 'uwb_' + SENSOR_TYPE + '_' + str(i)
    if SPAWN_MODEL:
        f.write('''\t<node pkg="gazebo_ros" type="spawn_model" name="spawn_{}" args="-sdf -model {} -x {} -y {} -z 1.0 -file $(arg uwb_{}_model)" />\n'''.format(
            uwb_name, uwb_name, str(x[i]), str(y[i]), SENSOR_TYPE))
    f.write('''\t<node pkg="tf" type="static_transform_publisher" name="{}" args="{} {} 0.0 0 0 0 1 map {} 100" />\n\n'''.format(
        uwb_name, str(x[i]), str(y[i]), uwb_name))

f.write('''</launch>''')
f.close()
