# =====================================================================  libs   ==========================================

# ==========================================
#    flask
# ==========================================
from threading import Lock
from flask import Flask, render_template, session, request, \
    copy_current_request_context
from flask_socketio import SocketIO, emit, join_room, leave_room, \
    close_room, rooms, disconnect
from engineio.payload import Payload
from flask_cors import CORS

# ==========================================
#   packages for ros
# ==========================================
import rclpy
import sys
from sensor_msgs.msg import Image
import cv2
import base64
import numpy as np
from rmoss_interfaces.msg import ChassisCmd
from rmoss_interfaces.msg import GimbalCmd
from rmoss_interfaces.msg import ShootCmd
import time

# =====================================================================  Variables   ==========================================

#

# ==========================================
#    robot state
# ==========================================

# 云台状态
gimbal_pitch = 0.0
gimbal_yaw = 0.0

# 移动速度
speed = 1.0

# 云台上界和下界
yaw_upper_bound = 1.37
yaw_lower_bound = -1.37
pitch_upper_bound = 0.9
pitch_lower_bound = -0.9

node = None
robot_name = None

# ==========================================
#    用于调试
# ==========================================
counter = 0
start_time = 0

# ==========================================
#    config
# ==========================================
async_mode = "threading"
Payload.max_decode_packets = 100
app = Flask(__name__)
app.config['SECRET_KEY'] = 'secret!'
CORS(app)
socketio = SocketIO(app, async_mode=async_mode)
img_thread = None
thread_lock = Lock()


# =====================================================================  Functions   ==========================================

# ==========================================
#    将图片处理成base64发送
# ==========================================
def img_callback(send_img_function):
    def func(msg: Image):
        # ==========================================
        #   发送帧率代码计算
        # 
        # global counter, start_time
        # if start_time == 0 or time.time() - start_time >1:
        #     print(counter)
        #     counter = 0
        #     start_time = time.time()
        # counter = counter + 1
        # ==========================================
        np_img = np.reshape(msg.data, (msg.height, msg.width, 3)).astype(np.uint8)
        img = cv2.cvtColor(np_img, cv2.COLOR_RGB2BGR)
        image = cv2.imencode('.jpg', img)[1]
        image_code = str(base64.b64encode(image))[2:-1]
        # print(image_code)
        send_img_function(image_code)

    return func

def send_img_function(img):
    # print("send")
    socketio.emit('image', {'img': img})


# ==========================================
#    发送血量、弹药量、图像数据给前端的线程
# ==========================================
def ros_img_thread(node, robot_name, send_img_function):
    img_sub = node.create_subscription(Image, '/%s/front_camera/image' % (robot_name), img_callback(send_img_function),
                                       rclpy.qos.qos_profile_sensor_data)
    rclpy.spin(node)
    node.destroy_node()

# ==========================================
#    返回页面
# ==========================================
@app.route('/')
def index():
    return render_template('index.html', async_mode=socketio.async_mode)

# ==========================================
#    监听加键鼠控制事件
# ==========================================
@socketio.on('control')
def control(message):
   
   pass

# ==========================================
#    连接事件
# ==========================================
@socketio.on('connect')
def connect():
    global img_thread, node, robot_name
    with thread_lock:
        if img_thread is None:
            img_thread = socketio.start_background_task(ros_img_thread, node, robot_name, send_img_function)

# ==========================================
#    断开连接
# ==========================================
@socketio.on('disconnect')
def test_disconnect():
    print('Client disconnected', request.sid)


# ==========================================
#    错误处理
# ==========================================
@socketio.on_error_default
def default_error_handler(e):
    print("======================= ERROR =======================")
    print(e)
    print(request.event["message"])
    print(request.event["args"])
    print("=====================================================")


if __name__ == '__main__':
    # global node, chassis_cmd_pub, gimbal_cmd_pub, shoot_cmd_pub

    #ROS2 Task
    rclpy.init()
    node=rclpy.create_node('player_web')
    #standard_robot_red1,standard_robot_blue1
    robot_name="standard_robot_red1"
    port=5000
    if len(sys.argv) == 3:
        robot_name = str(sys.argv[1])
        port = int(sys.argv[2])
    print("Robot name:",robot_name, "Port:",port)
    socketio.run(app, host='0.0.0.0')
