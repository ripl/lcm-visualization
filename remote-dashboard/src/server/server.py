#*******************************************************************************
# Project "Every car deserves a dashboard"
# Andrea F. Daniele
# Toyota Technological Institute at Chicago
# July 2017
#*******************************************************************************

import io
import sys
import lcm
import time
import signal
import psutil
import tornado
import numpy as np
import multiprocessing
from tornado.ioloop import IOLoop, PeriodicCallback
from tornado.websocket import WebSocketHandler, WebSocketClosedError
from transforms3d import euler
# import lcmtypes
from ros2lcm import rpy_t, mag_t, husky_status_t
from erlcm import velocity_msg_t
from kinect import frame_msg_t #TODO: remove

# server configuration
websocket_port = 8010
websocket_statusApp_frequencyHz = 10
websocket_statusApp_appName = 'husky_status'
websocket_cameraApp_frequencyHz = 8
websocket_cameraApp_appName = 'husky_camera'
compass_values = ['W','NW','N','NE','E','SE','S','SW']

# define temporary data containers
data_manager = multiprocessing.Manager()

default_message_info = {
    'temperature' : 20.0,
    'ram' : 0.0,
    'cpu' : 0.0,
    'gearbox' : 'P',
    'compass' : '-',
    'compass_heading' : 0.0,
    'gps_lat' : 0.0,
    'gps_lng' : 0.0,
    'linear_speed' : 0.0,
    'battery' : 0.0,
    'roll' : 0.0,
    'pitch' : 0.0,
    'lights_on' : False,
    'autopilot_on' : False,
    'faults_on' : False,
    'e_stop_on' : False
}

next_message_info = data_manager.dict()
next_message_info.update( default_message_info )

next_camera_frame = data_manager.dict()
next_camera_frame['data'] = 'empty_blob'

clients_connected = {'status' : [], 'camera' : []}

# define WebSocket handlers
class WS_Status_Handler(WebSocketHandler):
    def initialize(self): return

    def check_origin(self, origin): return True

    def open(self):
        # clients_connected.append( self )
        print 'A new client requested the app "status".'
        clients_connected['status'].append(self)

class WS_Camera_Handler(WebSocketHandler):
    def initialize(self): return

    def check_origin(self, origin): return True

    def open(self):
        # print 'New connection'
        print 'A new client requested the app "camera".'
        clients_connected['camera'].append(self)

def smooth( dictionary, key, new_value, wOld, wNew ):
    dictionary[key] = dictionary[key] * wOld + new_value * wNew

# define LCM handlers
class LCM_Status_Handler(multiprocessing.Process):
    def __init__(self, *args, **kwargs):
        super(LCM_Status_Handler, self).__init__()
        self.terminate = False
        # initialize LCM
        self.lcm = lcm.LCM()

    def terminate(self):
        self.terminate = True

    def rpy_handler_fcn(self, channel, data):
        msg = rpy_t.decode(data)
        smooth( next_message_info, 'roll', np.rad2deg(msg.r), .6, .4 )
        # next_message_info['roll'] = np.rad2deg(msg.r)
        smooth( next_message_info, 'pitch', np.rad2deg(msg.p), .6, .4 )
        # next_message_info['pitch'] = np.rad2deg(msg.p)

    def husky_status_handler_fcn(self, channel, data):
        msg = husky_status_t.decode(data)
        next_message_info['temperature'] = max(
            msg.left_driver_temp, msg.right_driver_temp,
            msg.left_motor_temp, msg.right_motor_temp
        )
        smooth( next_message_info, 'battery', msg.charge_estimate, .8, .2 )
        next_message_info['e_stop_on'] = (msg.e_stop or msg.lockout or msg.ros_pause)
        # opportunistically use the 1Hz frequency of the HUSKY_STATUS channel to get info about the machine
        smooth( next_message_info, 'cpu', psutil.cpu_percent()/100., .8, .2 )
        smooth( next_message_info, 'ram', psutil.virtual_memory().percent/100., .8, .2 )

    def velocity_cmd_handler_fcn(self, channel, data):
        msg = velocity_msg_t.decode(data)
        if next_message_info['e_stop_on']:
            next_message_info['gearbox'] = 'N'
        else:
            if msg.tv > 0:
                next_message_info['gearbox'] = 'D'
            elif msg.tv < 0:
                next_message_info['gearbox'] = 'R'
            else:
                if msg.rv != 0:
                    next_message_info['gearbox'] = 'D'
                else:
                    next_message_info['gearbox'] = 'P'
        if msg.tv == 0:
            next_message_info['linear_speed'] = 0.0
        else:
            smooth( next_message_info, 'linear_speed', np.abs(msg.tv)*3.6, .4, .6 ) # 1m/s = 3.6km/h

    def magnetometer_handler_fcn(self, channel, data):
        msg = mag_t.decode(data)
        angle = np.arctan2( msg.x, msg.y )
        angle = angle if angle >= 0 else 2*np.pi-angle
        angle_id = int(round( (angle / (2*np.pi) ) * 8.0 )) % 8  # 8 is the number of elements in compass_values
        next_message_info['compass'] = compass_values[ angle_id ]
        next_message_info['compass_heading'] = angle

    def run(self):
        self.lcm.subscribe("IMU_RPY", self.rpy_handler_fcn)
        self.lcm.subscribe("HUSKY_STATUS", self.husky_status_handler_fcn)
        self.lcm.subscribe("VELOCITY_CMD", self.velocity_cmd_handler_fcn)
        self.lcm.subscribe("MAGNETOMETER", self.magnetometer_handler_fcn)
        try:
            while True:
                self.lcm.handle()
        except KeyboardInterrupt:
            pass

class LCM_Camera_Handler(multiprocessing.Process):
    def __init__(self, *args, **kwargs):
        super(LCM_Camera_Handler, self).__init__()
        self.terminate = False
        # initialize LCM
        self.lcm = lcm.LCM()

    def terminate(self):
        self.terminate = True

    def chameleon_camera_handler_fcn(self, channel, data):
        msg = frame_msg_t.decode(data)
        rgb_blob = msg.image.image_data
        next_camera_frame['data'] = rgb_blob

    def run(self):
        self.lcm.subscribe("KINECT_FRAME", self.chameleon_camera_handler_fcn)
        try:
            while True:
                self.lcm.handle()
        except KeyboardInterrupt:
            pass


# run LCM Handlers threads
lcm_status_handler = LCM_Status_Handler()
lcm_camera_handler = LCM_Camera_Handler()
lcm_status_handler.start()
lcm_camera_handler.start()


# handle CTRL-C signal
def signal_handler(signal, frame):
    global status_publisher
    # terminate websocket server
    print 'Stopping the WebSocket server... ',; sys.stdout.flush()
    IOLoop.instance().stop()
    print 'Done!'
    # terminate the status publisher
    print 'Waiting for the LCM threads to terminate... ',; sys.stdout.flush()
    lcm_status_handler.terminate()
    lcm_camera_handler.terminate()
    lcm_status_handler.join()
    lcm_camera_handler.join()
    print 'Done!'
    # done!
    print 'Process ended!'
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)


# create and launch WebSocket server
app = tornado.web.Application(
    [(r"/%s" % websocket_statusApp_appName, WS_Status_Handler),
     (r"/%s" % websocket_cameraApp_appName, WS_Camera_Handler)]
)
http_server = tornado.httpserver.HTTPServer(app)
http_server.listen(websocket_port)

def status_schedule_func():
    data = dict( next_message_info )
    clients_updated = []
    for client in clients_connected['status']:
        try:
            client.write_message(data, binary=False)
            clients_updated.append(client)
        except WebSocketClosedError: pass
    clients_connected['status'] = clients_updated

def camera_schedule_func():
    data = next_camera_frame['data']
    clients_updated = []
    for client in clients_connected['camera']:
        try:
            client.write_message(data, binary=True)
            clients_updated.append(client)
        except WebSocketClosedError: pass
    clients_connected['camera'] = clients_updated


main_loop = IOLoop.instance()
status_sched = PeriodicCallback(status_schedule_func, 1000./websocket_statusApp_frequencyHz, io_loop=main_loop)
camera_sched = PeriodicCallback(camera_schedule_func, 1000./websocket_cameraApp_frequencyHz, io_loop=main_loop)
#start your period timer
status_sched.start()
camera_sched.start()
#start your loop
main_loop.start()
