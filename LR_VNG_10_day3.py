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
from pyzbar import pyzbar
import csv
from mavros_msgs.srv import CommandBool




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


# Reading QR codes
def image_callback_qr():
    data = rospy.wait_for_message('main_camera/image_raw', Image)
    cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')
    cv_image = cv_image[80:160, 100:220]

    # mask = cv.inRange(cv_image,(0,0,0),(200,200,200))
    # thresholded = cv.cvtColor(mask,cv.COLOR_GRAY2BGR)
    # inverted = 255-thresholded

    barcodes = pyzbar.decode(cv_image)
    # Checking the QR code readout
    if len(barcodes) == 0:
        flag_readable = False
    else:
        flag_readable = True
    # Getting the coordinates of the following QR code
    for barcode in barcodes:
        global coordinates
        coordinates = barcode.data.encode("utf-8")
    return flag_readable


# Checking readability
def check_readability(xc, yc):
    # print('I fly to the specified point')
    navigate_wait(x=xc, y=yc, z=heights[0], speed=1, frame_id='aruco_map')
    rospy.sleep(5)

    flag_result = False
    for height in heights:
        navigate_wait(x=xc, y=yc, z=height, speed=1, frame_id='aruco_map')
        rospy.sleep(3)
        for _ in range(5):
            flag_readable = image_callback_qr()
            rospy.sleep(0.3)
            if flag_readable == True:
                flag_result = True
                break
        if flag_readable == True:
            # print('The QR code is read, I output the text:')
            break
        # else:
            # print('The QR code is not read, I go down {} meters'.format(round(dz, 2)))
    return flag_result

with open("Report.txt", mode="w") as w_file:

# mode 'a'- just write
# mode 'w'- delete the previous file and write new

        fieldnames = [ 'x','y', 'z']
        file_writer = csv.DictWriter(w_file, fieldnames=fieldnames)
        file_writer.writeheader()

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

ret = []
def color_detect(cv_image):
    color = {'green': [40, 100, 60, 90, 255, 255, [0, 255, 0]]}
    colors_name = ['green']
    


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


def color_detect_black(cv_image):

    # cv_image = cv_image[110:220, 80:160]

    hsv_param = [0, 0, 0, 255, 255, 40]
    hsv_min = np.array((hsv_param[0], hsv_param[1], hsv_param[2]), np.uint8)
    hsv_max = np.array((hsv_param[3], hsv_param[4], hsv_param[5]), np.uint8)

    hsv = cv.cvtColor(cv_image, cv.COLOR_BGR2HSV)
    thresh = cv.inRange(hsv, hsv_min, hsv_max)


    new_image, contours, hierarchy = cv.findContours(thresh.copy(), cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    cv.drawContours(cv_image, contours, -1, (255,105,180), 3, cv.LINE_AA, hierarchy, 1)

    image_pub.publish(bridge.cv2_to_imgmsg(cv_image, 'bgr8'))

    img = np.zeros((cv_image.shape[0],cv_image.shape[1],3), np.uint8)
    img[:] = (255,255,255)
    cv.drawContours(img, contours, -1, (0,0,0), 3, cv.LINE_AA, hierarchy, 1)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    edges = cv.Canny(gray, 50, 150, apertureSize=3)
    minLineLength = 20
    lines = cv.HoughLinesP(image=edges, rho=0.02, theta=np.pi / 4, threshold=10, lines=np.array([]), minLineLength=minLineLength, maxLineGap=2)

    try:
        a, b, c = lines.shape
        imgg = np.zeros((img.shape[0],img.shape[1],3), np.uint8)
        imgg[:] = (255,255,255)
        for i in range(a):
            cv.line(imgg, (lines[i][0][0], lines[i][0][1]), (lines[i][0][2],  lines[i][0][3]), (0, 0, 0), 2, cv.LINE_AA)
        
        # image_pub_2.publish(bridge.cv2_to_imgmsg(imgg, 'bgr8'))
        




        contours = sorted(contours, key=cv.contourArea, reverse=True)
        contour = contours[0]
        (x,y,w,h)=cv.boundingRect(contour)
        
        if h < w:
            imgg_right = imgg[0:imgg.shape[0], imgg.shape[1]//2:imgg.shape[1]]
            imgg_left = imgg[0:imgg.shape[0], 0:imgg.shape[1]//2]
            # image_pub_3.publish(bridge.cv2_to_imgmsg(imgg_right, 'bgr8'))
            # image_pub_4.publish(bridge.cv2_to_imgmsg(imgg_left, 'bgr8'))
            hsv_1 = cv.cvtColor(imgg_right, cv.COLOR_BGR2HSV)
            thresh_1 = cv.inRange(hsv_1, hsv_min, hsv_max)
            hsv_2 = cv.cvtColor(imgg_left, cv.COLOR_BGR2HSV)
            thresh_2 = cv.inRange(hsv_2, hsv_min, hsv_max)

            moments_1 = cv.moments(thresh_1, 0)
            dArea_1 = moments_1['m00']
            moments_2 = cv.moments(thresh_2, 0)
            dArea_2 = moments_2['m00']

            if dArea_1 > dArea_2:
                # n = 'Sector D'
                n = 'Sector A'
            else:
                # n = 'Sector C'
                n = 'Sector B'
        else:
            imgg_up = imgg[0:imgg.shape[0]//2, 0:imgg.shape[1]]
            imgg_down = imgg[imgg.shape[0]//2:imgg.shape[0], 0:imgg.shape[1]]
            # image_pub_3.publish(bridge.cv2_to_imgmsg(imgg_up, 'bgr8'))
            # image_pub_4.publish(bridge.cv2_to_imgmsg(imgg_down, 'bgr8'))
            hsv_1 = cv.cvtColor(imgg_up, cv.COLOR_BGR2HSV)
            thresh_1 = cv.inRange(hsv_1, hsv_min, hsv_max)
            hsv_2 = cv.cvtColor(imgg_down, cv.COLOR_BGR2HSV)
            thresh_2 = cv.inRange(hsv_2, hsv_min, hsv_max)
                    
            moments_1 = cv.moments(thresh_1, 0)
            dArea_1 = moments_1['m00']
            moments_2 = cv.moments(thresh_2, 0)
            dArea_2 = moments_2['m00']

            if dArea_1 > dArea_2:
                # n = 'Sector B'
                n = 'Sector D'
            else:
                # n = 'Sector A'
                n = 'Sector C'

        return n
        # print n
    except AttributeError:
        pass


sect = ''
ret = []
def image_callback(data):
    global sect

    cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')
    sect = color_detect_black(cv_image)

    global ret
    ret = color_detect(cv_image)

    out.write(cv_image)





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



start_times = rospy.get_rostime()


navigate_wait(x=0, y=0, z=1, speed=0.5, frame_id='body', auto_arm=True)
navigate_wait(x=0, y=0, z=1, yaw=0, speed=0.5, frame_id='aruco_map')
navigate_wait(x=0, y=0, z=1, speed=0.5, frame_id='aruco_map')
# navigate_wait(x=1.2, y=1.2, z=1, yaw=math.pi/2, speed=0.5, frame_id='aruco_map')
# navigate_wait(x=1.2, y=1.2, z=0.4, yaw=math.pi/2, speed=0.5, frame_id='aruco_map')


##################################################################

tel_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, txt_file)


fourcc = cv.VideoWriter_fourcc(*'mp4v')
out = cv.VideoWriter('video_capture.mp4', fourcc, 20.0, (320, 240))
image_sub = rospy.Subscriber('main_camera/image_raw', Image, image_callback)
image_pub = rospy.Publisher('~Detect', Image, queue_size=10)



# Cruise altitude
z_cruise = 1
# Minimum QR code reading height
z_min = 0.4
# Number of attempts to read the QR code
n_attempts = 7
# Changing the height for a new attempt
dz = (z_cruise - z_min) / n_attempts

# Coordinates of 1 QR code
xc = 0.4
yc = 0.8


# Creating a list of height values for reading QR codes
heights = []
for i in range(n_attempts + 1):
    new_height = z_cruise - i * dz
    heights.append(new_height)


# Flying to the QR code
flag_result = check_readability(xc, yc)
# Checking the QR code reading after all attempts
# If it didn't work out:
if flag_result == False:
    # Error message
    land_wait()
# If it worked
else:
    # Output of the QR code text
    inf = coordinates
    list_inf = []
    for i in range(len(inf)):
        try:
            if inf[i] != ' ' and inf[i] != '' and inf[i] != '\n' and inf[i] != '.' and inf[i-1] != '.':
                try:
                    if inf[i+1] == '.':
                        s = inf[i] + inf[i+1] + inf[i+2]
                        list_inf.append(float(s))
                    else:
                        list_inf.append(float(inf[i]))
                except IndexError:
                    list_inf.append(float(inf[i]))
        except IndexError:
            try:
                if inf[i+1] == '.':
                    s = inf[i] + inf[i+1] + inf[i+2]
                    list_inf.append(float(s))
                else:
                    list_inf.append(float(inf[i]))
            except IndexError:
                list_inf.append(float(inf[i]))

# x1, y1, x2, y2 = list_inf[0], list_inf[1], list_inf[2], list_inf[3]
xs, ys = list_inf[len(list_inf)-3], list_inf[len(list_inf)-2]
N = int(list_inf[len(list_inf)-1])
for i in range(0, len(list_inf)-3, 2):
    x1, y1 = list_inf[i], list_inf[i+1]
    print 'Column area x={}, y={}'.format(x1, y1)
print 'Navigation area x={}, y={}'.format(xs, ys)
print 'Order number: {}'.format(N)    

# print 'Column area x={}, y={}'.format(x1, y1)
# print 'Column area x={}, y={}'.format(x2, y2)
# print 'Navigation area x={}, y={}'.format(xs, ys)
# print 'Order number: {}'.format(N)




























navigate_wait(x=0.4, y=0.8, z=1, speed=0.5, frame_id='aruco_map')
navigate_wait(x=2.4, y=2.4, z=1, speed=0.5, frame_id='aruco_map')
navigate_wait(x=xs-0.2, y=ys, z=1, speed=0.5, frame_id='aruco_map')
navigate_wait(x=xs-0.2, y=ys, z=0.7, speed=0.5, frame_id='aruco_map')


rospy.sleep(6)
n = sect

kkk = 0.1
zzz = 0.5
for _ in range(5):
    if n == None:
        zzz += kkk
        navigate_wait(x=xs-0.2, y=ys, z=zzz, speed=0.5, frame_id='aruco_map')
    else:
        break


print n, 'required'

navigate_wait(x=xs-0.2, y=ys-0.2, z=1.2, speed=0.5, frame_id='aruco_map')
rospy.sleep(3)

flag_n = True
x_cop = 160
y_cop = 120
while flag_n:
    # print ret
    if ret == []:
        pose = rospy.wait_for_message('/mavros/local_position/pose', PoseStamped)
        new_pose = tf_buffer.transform(pose, 'aruco_map', rospy.Duration(0.2))
        x_map = new_pose.pose.position.x
        y_map = new_pose.pose.position.y
        # navigate_wait(x=x_map+0.4, y=y_map+0.4, z=1, speed=0.5, frame_id='aruco_map')
        # print('1')
        if n == 'Sector A':
            navigate_wait(x=x_map, y=y_map+0.4, z=1.2, speed=0.3, frame_id='aruco_map')
        if n == 'Sector B':
            navigate_wait(x=x_map, y=y_map-0.4, z=1.2, speed=0.3, frame_id='aruco_map')
        if n == 'Sector C':
            navigate_wait(x=x_map+0.4, y=y_map, z=1.2, speed=0.3, frame_id='aruco_map')
        if n == 'Sector D':
            navigate_wait(x=x_map-0.4, y=y_map, z=1.2, speed=0.3, frame_id='aruco_map')
    else:
        cm = ret[0]
        x_cm = cm[0][0]
        y_cm = cm[0][1]
        d = 0.06
        pose = rospy.wait_for_message('/mavros/local_position/pose', PoseStamped)
        new_pose = tf_buffer.transform(pose, 'aruco_map', rospy.Duration(0.2))
        x_map = new_pose.pose.position.x
        y_map = new_pose.pose.position.y
        tang = y_cm / x_cm

        if y_cm < y_cop:
            x = x_map + d
        if y_cm >= y_cop:
            x = x_map - d
        if x_cm < x_cop:
            y = y_map + (d*tang)
        if x_cm >= x_cop:
            y = y_map - (d*tang)

        dist = rospy.wait_for_message('/rangefinder/range', Range)
        dist = dist.range
        if dist<0.3:
            land()
            break

        if abs(x_cop - x_cm) < 30 and abs(y_cop - y_cm) < 30:
            flag_n = False
            land()
            rospy.sleep(5)
            # navigate_wait(x=x, y=y, z=dist-0.1, speed=1, frame_id='aruco_map')
        else:
            navigate_wait(x=x, y=y, z=1.2, speed=1, frame_id='aruco_map')

        # print x, y, dist
        # navigate_wait(x=x, y=y, z=dist+0.1, speed=1, frame_id='aruco_map')
        # print('2')
    rospy.sleep(0.5)



land()
rospy.sleep(2)

print N, 'delivered in', n 

rospy.sleep(2)

navigate_wait(x=0, y=0, z=1, speed=0.5, frame_id='body', auto_arm=True)
navigate_wait(x=2.4, y=2.4, z=1, speed=0.5, frame_id='aruco_map')
navigate_wait(x=0, y=0, z=1, speed=0.5, frame_id='aruco_map')
navigate_wait(x=0, y=0, z=0.6, speed=0.5, frame_id='aruco_map')

land_wait()
times = rospy.get_rostime()
time_from_the_beginning = times

m = round(time_from_the_beginning/60, 0)
s = round(time_from_the_beginning%60, 0)


print N, 'delivered in', n, 'for', m, 'min', s, 'sec'






rospy.spin()




