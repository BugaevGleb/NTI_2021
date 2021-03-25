import rospy
import math
from math import sqrt
from rosgraph_msgs.msg import Clock
import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from sensor_msgs.msg import Range
from geometry_msgs.msg import PoseStamped
from clover import srv
from std_srvs.srv import Trigger
import tf2_ros
import tf2_geometry_msgs
from aruco_pose.msg import MarkerArray
from std_msgs.msg import String
import csv



rospy.init_node('flight')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)

bridge = CvBridge()

tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)

with open("Report.txt", mode="w") as w_file:

# mode 'a'- just write
# mode 'w'- delete the previous file and write new

        fieldnames = [ 'x','y', 'z']
        file_writer = csv.DictWriter(w_file, fieldnames=fieldnames)
        file_writer.writeheader()



angle = 99999999

def color_detect_black(cv_image):
    global angle

    original = cv_image

    hsv = cv.cvtColor(original, cv.COLOR_BGR2HSV)
    (_ret, threshold) = cv.threshold(hsv[:,:,1], 90, 255, cv.THRESH_OTSU)
    dist = cv.distanceTransform(threshold, cv.DIST_L2, cv.DIST_MASK_PRECISE)
    idx=np.argmax(dist)
    y,x=np.unravel_index(idx, dist.shape) #corner position
    # color=original[y,x,:]
    M = cv.moments(threshold)
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])

    angle = 180*np.arctan2(x-cX, y-cY)/np.pi
    return angle
    # print(180*np.arctan2(x-cX, y-cY)/np.pi, 'degrees')
    # print(color[::-1], 'rgb color')

        
ang = 9999
def image_callback(data):
    global ang

    cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')
    ang = color_detect_black(cv_image)
    out.write(cv_image)




def navigate_wait(x=0, y=0, z=0, yaw=float('nan'), speed=0.5, frame_id='', auto_arm=False, tolerance=0.2):
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)

def land_wait():
    land()
    while get_telemetry().armed:
        rospy.sleep(0.2)

# def landing_to_box():
#     global flag
#     pose = rospy.wait_for_message('/mavros/local_position/pose', PoseStamped)
#     new_pose = tf_buffer.transform(pose, 'aruco_map', rospy.Duration(0.2))
#     x = new_pose.pose.position.x
#     y = new_pose.pose.position.y
#     z = new_pose.pose.position.z
#     dist = rospy.wait_for_message('/rangefinder/range', Range)
#     dist = dist.range
#     print dist
#     if dist < 0.14:
#         # arming(value=False)
#         print('5')
#         land()
#         flag = False
#     else:
#         navigate_wait(x=x, y=y, z=dist-0.1, speed=0.5, frame_id='aruco_map')
#         print('4')
#     rospy.sleep(1)

def color_detect(cv_image):
    color = {'green': [40, 100, 60, 90, 255, 255, [0, 255, 0]]}
    colors_name = ['green']
    
    ret = []

    for name in colors_name:
        hsv_param = color[name]
        hsv_min = np.array((hsv_param[0], hsv_param[1], hsv_param[2]), np.uint8)
        hsv_max = np.array((hsv_param[3], hsv_param[4], hsv_param[5]), np.uint8)

        hsv = cv.cvtColor(cv_image, cv.COLOR_BGR2HSV)
        thresh = cv.inRange(hsv, hsv_min, hsv_max)
        new_image, contours0, hierarchy = cv.findContours(thresh.copy(), cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        
        for cnt in contours0:
            rect = cv.minAreaRect(cnt)
            box = cv.boxPoints(rect)
            box = np.int0(box)

            thresh_new = cv.GaussianBlur(thresh, (5, 5), 2)
            rows = thresh_new.shape[0]
            circles = []
            circles = cv.HoughCircles(thresh_new, cv.HOUGH_GRADIENT, 1, rows / 8,
                                    param1=100, param2=30,
                                    minRadius=1, maxRadius=50)

            if circles is not None:
                circles = np.uint16(np.around(circles))
                for i in circles[0, :]:
                    center = [i[0], i[1]]
                    ret.append([center, 'circle', name])

            if sqrt((box[0][0] - box[2][0])**2 + (box[0][1] - box[2][1])**2) > 20:
                
                min_x = box[:, 0].min()
                max_x = box[:, 0].max()
                min_y = box[:, 1].min()
                max_y = box[:, 1].max()

                new_min_y = min_y-20 if min_y-20 >= 0 else 0
                new_max_y = max_y+20 if max_y+20 >= 0 else 0
                new_min_x = min_x-20 if min_x-20 >= 0 else 0
                new_max_x = max_x+20 if max_x+20 >= 0 else 0

                thresh_new = thresh[new_min_y:new_max_y, new_min_x:new_max_x]

                moments = cv.moments(thresh_new, 1)
                dM01 = moments['m01']
                dM10 = moments['m10']
                dArea = moments['m00']

                x = int(dM10 / dArea) + new_min_x
                y = int(dM01 / dArea) + new_min_y

                k = 0
                try:
                    for i in circles[0, :]:
                        if  abs(i[0] - x) < 10 and abs(i[1] - y) < 10:
                            k += 1
                except TypeError:
                    k == 0

                if k == 0:
                    ret.append([[x, y], 'rectangle', name])
    return ret

###################



def txt_file(data):
    x1 = data.pose.position.x
    y1 = data.pose.position.y
    z1 = data.pose.position.z

    x1 = round(x1, 3)
    y1 = round(y1, 3)
    z1 = round(z1, 3)

    with open("Report.txt", mode="a") as w_file:
        fieldnames = ['x', 'y', 'z']
        file_writer = csv.DictWriter(w_file, fieldnames=fieldnames, lineterminator = '\n')
        file_writer.writerow({'x': x1, 'y': y1, 'z': z1})


ret = []

start_times = rospy.get_rostime()

pi2 = math.pi / 2

navigate_wait(x=0, y=0, z=1, speed=1, frame_id='body', auto_arm=True)
navigate_wait(x=0, y=0, z=1, speed=1, frame_id='aruco_map')
navigate_wait(x=1.2, y=1.2, z=1, speed=1, frame_id='aruco_map')


##################################################################

tel_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, txt_file)


fourcc = cv.VideoWriter_fourcc(*'mp4v')
out = cv.VideoWriter('video_capture.mp4', fourcc, 20.0, (320, 240))
image_sub = rospy.Subscriber('main_camera/image_raw', Image, image_callback)

# for _ in range(20):

#     n = ''
#     if ang > -45 and ang <= 45:
#         n = 'Sector A'
#     if (ang < -135 and ang >= -180) or (ang > 135 and ang <= 180):
#         n = 'Sector B'
#     if ang >= -135 and ang <= -45:
#         n = 'Sector C'
#     if ang > 45 and ang <= 135:
#         n = 'Sector D'

    # print(ang)
    # print(n)

summ = 0
nnn = 0
for _ in range(20):
    if ang != 9999:
        summ += ang
        nnn += 1
    rospy.sleep(0.5)


aaa = summ/nnn
n = ''
if aaa > -45 and aaa <= 45:
    n = 'Sector B'
if (aaa < -135 and aaa >= -180) or (aaa > 135 and aaa <= 180):
    n = 'Sector A'
if aaa >= -135 and aaa <= -45:
    n = 'Sector C'
if aaa > 45 and aaa <= 135:
    n = 'Sector D'

    # print(ang)
    # print(n)












print(n, 'required')

navigate_wait(x=1.2, y=1.2, z=1, speed=1, frame_id='aruco_map')

flag_n = True
x_cop = 160
y_cop = 120
while flag_n:
    if ret == []:
        pose = rospy.wait_for_message('/mavros/local_position/pose', PoseStamped)
        new_pose = tf_buffer.transform(pose, 'aruco_map', rospy.Duration(0.2))
        x_map = new_pose.pose.position.x
        y_map = new_pose.pose.position.y
        if n == 'Sector A':
            navigate_wait(x=x_map+0.4, y=y_map, z=1.2, speed=0.5, frame_id='aruco_map')
        if n == 'Sector B':
            navigate_wait(x=x_map-0.4, y=y_map, z=1.2, speed=0.5, frame_id='aruco_map')
        if n == 'Sector C':
            navigate_wait(x=x_map, y=y_map+0.4, z=1.2, speed=0.5, frame_id='aruco_map')
        if n == 'Sector D':
            navigate_wait(x=x_map, y=y_map-0.4, z=1.2, speed=0.5, frame_id='aruco_map')
        # print('1')
    else:
        cm = ret[0]
        x_cm = cm[0][0]
        y_cm = cm[0][1]
        d = 0.06
        pose = rospy.wait_for_message('/mavros/local_position/pose', PoseStamped)
        new_pose = tf_buffer.transform(pose, 'aruco_map', rospy.Duration(0.2))
        x_map = new_pose.pose.position.x
        y_map = new_pose.pose.position.y
        tang = x_cm / y_cm

        if y_cm < y_cop:
            x = x_map + d
        if y_cm > y_cop:
            x = x_map - d
        if x_cm < x_cop:
            y = y_map + (d*tang)
        if x_cm > x_cop:
            y = y_map - (d*tang)
        # print x, y
        if abs(x_cop - x_cm) < 5 and abs(y_cop - y_cm) < 5:
            flag_n = False

        dist = rospy.wait_for_message('/rangefinder/range', Range)
        dist = dist.range
        # print dist

        navigate_wait(x=x, y=y, z=dist, speed=1, frame_id='aruco_map')
        # print('2')
    rospy.sleep(0.5)

land()
rospy.sleep(2)

N = 3
print(N, 'delivered in', n)

rospy.sleep(3)

navigate_wait(x=0, y=0, z=1, speed=1, frame_id='body', auto_arm=True)
navigate_wait(x=0, y=0, z=1, speed=1, frame_id='aruco_map')
navigate_wait(x=0, y=0, z=0.6, speed=1, frame_id='aruco_map')

land()
times = rospy.get_rostime()
time_from_the_beginning = times - start_time

m = round(time_from_the_beginning/60, 0)
s = round(time_from_the_beginning%60, 0)


print(N, 'delivered in', n, 'for', m, 'min', s, 'sec')






rospy.spin()




