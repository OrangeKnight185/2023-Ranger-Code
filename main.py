import numpy as np
import logging
import time
import socket
import PySpin
import cv2
import sys
import csv
import os
from struct import unpack
from queue import LifoQueue
from ctypes import c_double
from multiprocessing import Array, Process
from controller import XboxController
from tools import daemon
from PyQt5 import QtGui
from PyQt5.QtWidgets import QWidget, QMainWindow, QLabel, QHBoxLayout, QApplication
from PyQt5.QtGui import QPixmap
from PyQt5.QtCore import pyqtSlot, pyqtSignal, Qt, QThread, QObject
from timeit import default_timer as timer

NUM_OF_THRUSTERS = 8

T0D = -1.0
T1D = 1.0
T2D = 1.0
T3D = -1.0
T4D = 1.0
T5D = -1.0
T6D = -1.0
T7D = 1.0

STRG_THRUSTER_RATIO = 0.4663076582
WEAK_THRUSTER_RATIO = 1.0

THRUSTER_KEY_MATRIX = np.array([
    [ T0D * STRG_THRUSTER_RATIO,  T1D * STRG_THRUSTER_RATIO,  T2D,  T3D, -T4D * STRG_THRUSTER_RATIO, -T5D * STRG_THRUSTER_RATIO, -T6D, -T7D], # forward
    [ T0D, -T1D,  T2D * STRG_THRUSTER_RATIO, -T3D * STRG_THRUSTER_RATIO,  T4D, -T5D,  T6D * STRG_THRUSTER_RATIO, -T7D * STRG_THRUSTER_RATIO], # right
    [ T0D,  T1D, -T2D, -T3D,  T4D,  T5D, -T6D, -T7D], # down
    [-T0D,  T1D,  T2D, -T3D,  T4D, -T5D, -T6D,  T7D], # yaw right
    [-T0D * STRG_THRUSTER_RATIO, -T1D * STRG_THRUSTER_RATIO,  T2D,  T3D,  T4D * STRG_THRUSTER_RATIO,  T5D * STRG_THRUSTER_RATIO, -T6D, -T7D], # pitch up
    [ T0D, -T1D, -T2D * STRG_THRUSTER_RATIO,  T3D * STRG_THRUSTER_RATIO,  T4D, -T5D, -T6D * STRG_THRUSTER_RATIO,  T7D * STRG_THRUSTER_RATIO]]).T # roll right
# T:  0     1     2     3     4     5     6     7

#THRUST_MAX_KG = 1
THRUST_MAX_KG = 2.85

THRUSTER_DX = 90.32 / 1000.0
THRUSTER_DY = 90.46 / 1000.0
THRUSTER_DZ = 86.56 / 1000.0
THRUSTER_RADIUS = np.sqrt(THRUSTER_DX ** 2 + THRUSTER_DY ** 2 + THRUSTER_DZ ** 2)

THRUSTER_DISPLACEMENT_MATRIX = np.array([
    [ THRUSTER_DX,  THRUSTER_DY, -THRUSTER_DZ], # T0
    [ THRUSTER_DX, -THRUSTER_DY, -THRUSTER_DZ], # T1
    [ THRUSTER_DX,  THRUSTER_DY,  THRUSTER_DZ], # T2
    [ THRUSTER_DX, -THRUSTER_DY,  THRUSTER_DZ], # T3
    [-THRUSTER_DX,  THRUSTER_DY, -THRUSTER_DZ], # T4
    [-THRUSTER_DX, -THRUSTER_DY, -THRUSTER_DZ], # T5
    [-THRUSTER_DX,  THRUSTER_DY,  THRUSTER_DZ], # T6
    [-THRUSTER_DX, -THRUSTER_DY,  THRUSTER_DZ]]) # T7

THRUSTER_DIRECTION_COMPONENT = np.sqrt(3) / 3
T0X = 0.7848855672
T0Y = 0.3659981508
T0Z = 0.5

THRUSTER_DIRECTION_MATRIX = np.array([
    [ T0X,  T0Y,  T0Z], # T0
    [ T0X, -T0Y,  T0Z], # T1
    [ T0Y,  T0X, -T0Z], # T2
    [ T0Y, -T0X, -T0Z], # T3
    [-T0X,  T0Y,  T0Z], # T4
    [-T0X, -T0Y,  T0Z], # T5
    [-T0Y,  T0X, -T0Z], # T6
    [-T0Y, -T0X, -T0Z]]) # T7

thruster_torque_array = []
for i in range(0, NUM_OF_THRUSTERS):
    thruster_torque_array.append(np.cross(THRUSTER_DISPLACEMENT_MATRIX[i], THRUSTER_DIRECTION_MATRIX[i]))
THRUSTER_TORQUE_MATRIX = np.array(thruster_torque_array).T

def torques_from_thrusters(thrusts: np.ndarray):
    return THRUSTER_TORQUE_MATRIX @ thrusts.T

def thrusts_from_thrusters(thrusts: np.ndarray):
    return THRUSTER_DIRECTION_MATRIX @ thrusts.T

def normalize(value, range_min, range_max, new_min, new_max):
    return (value - range_min) * (new_max - new_min) / (range_max - range_min) + new_min

def normalize_thrusters(raw_thrusters: np.ndarray):

    if raw_thrusters.max() > abs(raw_thrusters.min()):
        largest_element = raw_thrusters.max()
    else:
        largest_element = abs(raw_thrusters.min())

    if largest_element > 1.0:
        raw_thrusters = raw_thrusters / largest_element
    
    return raw_thrusters

def thruster_percents(desired_movement: np.ndarray):
    thruster_values = THRUSTER_KEY_MATRIX @ desired_movement.T
    return normalize_thrusters(thruster_values.T)

def thrust_to_pwm(thrust):
    if thrust > 0:
        pwm = -10.651 * (thrust ** 2) + 131.24 * thrust + 1546.9
    elif thrust < 0:
        pwm = 16.079 * (thrust ** 2) + 164.41 * thrust + 1453.5
    else:
        pwm = 1500
    return round(pwm)
v_thrust_to_pwm = np.vectorize(thrust_to_pwm)

def percent_to_pwm(thruster_percent):
    return thrust_to_pwm(thruster_percent * THRUST_MAX_KG)
v_percent_to_pwm = np.vectorize(percent_to_pwm)

def percent_to_thrust(thruster_percents):
    return thruster_percents * THRUST_MAX_KG
v_percent_to_thrust = np.vectorize(percent_to_thrust)

def closest_val(l, K):
    return min(range(len(l)), key = lambda i: abs(float(l[i])-K))

rows = []
force_sheet = []
with open(r'T200.csv', 'r') as file:
    next(file)
    csvreader = csv.reader(file)
    for row in csvreader:
        rows.append(row)
        force_sheet.append(row[2])

NEWTON_CURRENT = 6
SERVO_CURRENT = 5
EXTRA_CURRENT = 3
CORRECTION_RESERVED_CURRENT = 5
TOTAL_CURRENT = 20
CURRENT_REDUCTION_SCALAR = 0.95
def current_check(thrusts_vector):
    current = 0
    indices = []
    for i, thrust in enumerate(thrusts_vector):
        indices.append(closest_val(force_sheet, thrust))
    for i in indices:
        current += float(rows[i][1])
    return current

precise_mode = False
square_inputs = False

WINDOW_ARRAY_SIZE = 830 * 832 * 3

CAM_ONE_ARRAY = Array(c_double, WINDOW_ARRAY_SIZE)
CAM_TWO_ARRAY = Array(c_double, WINDOW_ARRAY_SIZE)

CAMERA_ONE_SERIAL = "21205668"
CAMERA_TWO_SERIAL = "19069560"

def cam_process(image_array, serial_number):
    camera_application = QApplication(sys.argv)
    driver_window = CameraWindow(image_array, serial_number)
    driver_window.show()
    sys.exit(camera_application.exec())

INVERT_RSY = False
DEADZONE = True
DEADZONE_RADIUS = .1
VERTICAL_SCALE_FACTOR = 1
ROLL_SCALE_FACTOR = 1
PITCH_SCALE_FACTOR = 1

DO_IMU_HANDLING = False
DO_ACCEL_HANDLING = True
DO_GYRO_HANDLING = True

ROBOT_MASS = 9.8 + 1
ROBOT_INERTIA = (np.array([
    [1026922.88, 30776.55, -15817.68],
    [30776.55, 1536734.64, -13389.12],
    [-15817.68, -13389.12, 1464790.06]]) * (10 ** (-7))) @ np.array([
        [0, -1, 0],
        [1, 0, 0],
        [0, 0, -1]])

CAM_SPEED = 5

IP = "10.11.57.2"
PORT = 8888
PACKET_SIZE = 8
ENCODING = "utf-8"
QUEUE_WARN_SIZE = 100

thrusters_enabled = True

def centered_second_difference(p1, p2, p3, t1, t2):
    return (p1 - 2 * p2 + p3) / (t1 * t2)

def backwards_difference(p1, p2, t):
    return (p1 / p2) / t

class LeakError(Exception):
     def __init__(self):
         super.__init__("LEAK DETECTED")

class ImageEventHandler(PySpin.ImageEventHandler):
    #this class exists so the CPU only does work when there's a new image

    def __init__(self, parent_class):
        super(ImageEventHandler, self).__init__()
        self.parent_class = parent_class

    def OnImageEvent(self, image):
        #gets called by the camera every time there's a new image
        self.image_cv2 = image.Convert(PySpin.PixelFormat_BGR8, PySpin.HQ_LINEAR).GetData().reshape(830, 832, 3) #convert the image to color, then to openCV format #NOTE: Does the reshape function need to have the correct image sizes, if so find a way to do it.
        # cv2.putText(self.image_cv2, str(round(time.time() - self.parent_class.video_start, 4)) + " s", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 1, cv2.LINE_AA)
        image.Release() #clean up the grabbed image
        self.parent_class.change_pixmap_signal.emit(self.image_cv2) #give the thread a new image

class VideoThread(QObject):
    change_pixmap_signal = pyqtSignal(np.ndarray)

    video_start = time.time()

    def __init__(self, serial_num):
        super().__init__()
        self._run_flag = True
        #find the correct cameras  
        self.PySpin_system = PySpin.System.GetInstance()
        self.cam_list = self.PySpin_system.GetCameras()
        self.cam1 = self.cam_list.GetBySerial(serial_num) #  21205668 or 19069560
        
        #initialize them
        self.cam1.Init()
    
        #initialize the event handler
        self.image_event_handler = ImageEventHandler(self)
        self.cam1.RegisterEventHandler(self.image_event_handler)
            
        self.cam1.UserSetSelector.SetValue(PySpin.UserSetSelector_UserSet0)
        self.cam1.UserSetLoad()
        self.cam1.TLStream.StreamBufferHandlingMode.SetValue(PySpin.StreamBufferHandlingMode_NewestOnly)

    def run(self):
        self.cam1.BeginAcquisition()

        while self._run_flag:
            time.sleep(.1) #needed for the OnImageEvent to work

        #shut down all the capture infrastructure. This prevents a crash on close
        self.cam1.EndAcquisition()
        self.cam1.UnregisterEventHandler(self.image_event_handler)

        self.cam1.DeInit()
        del self.cam1
        
        self.cam_list.Clear()
        self.PySpin_system.ReleaseInstance()

    def stop(self):
        # Sets run flag to False and waits for thread to finish
        self._run_flag = False

class CameraWindow(QMainWindow):
    def __init__(self, a, cam_serial):
        super().__init__()

        self.img_global = a

        # create the label that holds the image
        self.resize(830, 832)

        self.center_widget = QWidget()
        self.setCentralWidget(self.center_widget)

        self.image_label = QLabel()
        self.image_label.resize(830, 832)

        # create a vertical box layout and add the two labels
        vbox = QHBoxLayout()
        vbox.addWidget(self.image_label)
        # set the vbox layout as the widgets layout
        self.center_widget.setLayout(vbox)

        self.camera_thread = QThread()
        self.camera = VideoThread(cam_serial)
        self.camera.moveToThread(self.camera_thread)
        self.camera_thread.started.connect(self.camera.run)
        self.camera.change_pixmap_signal.connect(self.camera_callback)
        self.camera_thread.start()

    @pyqtSlot(np.ndarray)
    def camera_callback(self, cv_img):
        self.image_label.setPixmap(self.convert_cv_qt(cv_img))

    def convert_cv_qt(self, cv_img):
        """Convert from an opencv image to QPixmap"""
        rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        convert_to_Qt_format = QtGui.QImage(rgb_image.data, w, h, bytes_per_line, QtGui.QImage.Format_RGB888)
        p = convert_to_Qt_format.scaled(h, w, Qt.KeepAspectRatio)
        return QPixmap.fromImage(p)

    def closeEvent(self, event):
        self.camera.stop()
        time.sleep(.1)
        self.camera_thread.quit()
        time.sleep(.1)
        event.accept()

class Acceleration:
    def __init__(self, x = 0, y = 0, z = 0):
        self.x = x
        self.y = y
        self.z = z

class Angles:
    def __init__(self, yaw = 0, pitch = 0, roll = 0):
        self.yaw = [yaw]
        self.pitch = [pitch]
        self.roll = [roll]
        self.time = timer()

class Networking:
    accel_data = LifoQueue()
    gyro_data = LifoQueue()
    depth_data = LifoQueue()

    def __init__(self):
        self.arduino = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.arduino.setblocking(False)
        self.arduino.settimeout(False)

        time.sleep(.1)

    def construct_thruster_packet(self, thruster_pwm_vector):
        self.packet = b'T'
        for pwm in thruster_pwm_vector:
            self.first_half = pwm // (16 ** 2)
            self.second_half = pwm % (16 ** 2)
            self.packet += self.first_half.item().to_bytes(1, "big")
            self.packet += self.second_half.item().to_bytes(1, "big")
        return self.packet
    
    def construct_manip_packet(self, manip_num, pwm):
        self.packet = b'M'
        self.packet += manip_num
        self.first_half = pwm // (16 ** 2)
        self.second_half = pwm % (16 ** 2)
        self.packet += self.first_half.to_bytes(1, "big")
        self.packet += self.second_half.to_bytes(1, "big")
        return self.packet
    
    def construct_camera_packet(self, pwm):
        self.packet = b'C'
        self.first_half = pwm // (16 ** 2)
        self.second_half = pwm % (16 ** 2)
        self.packet += self.first_half.to_bytes(1, "big")
        self.packet += self.second_half.to_bytes(1, "big")
        return self.packet

    def send(self, packet):
        self.arduino.sendto(packet, (IP, PORT))

    def scan(self):
        self.acceleration = Acceleration()
        self.angles = Angles()
        try:
            packet, address = self.arduino.recvfrom(2048)
            if packet != None:
                if chr(packet[0]) == "A":
                    self.acceleration.x = unpack(">h", packet[1:3]) / 100.0
                    self.acceleration.y = unpack(">h", packet[3:5]) / 100.0
                    self.acceleration.z = unpack(">h", bytes([packet[5], packet[6]])) / 100.0
                    self.accel_data.put(self.acceleration)
                elif chr(packet[0]) == "G":
                    self.angles.yaw = unpack(">h", packet[1:3]) / 100.0
                    self.angles.pitch = unpack(">h", packet[3:5]) / 100.0
                    self.angles.roll = unpack(">h", bytes([packet[5], packet[6]])) / 100.0
                    self.angles.time = timer()
                    self.gyro_data.put(self.angles)
                elif chr(packet[0]) == "D":
                    self.depth = packet[1] + packet[2] / 100.0
                    self.depth_data.put(self.depth)
                elif chr(packet[0]) == "L":
                    raise(LeakError)
        except (OSError, BlockingIOError) as e:
            pass

def main():
    global precise_mode

    thrust_per_thruster = np.array([0, 0, 0, 0, 0, 0, 0, 0])
    found_controller = False
    networking = Networking()
    previous_ls = 0
    previous_dx = 0
    red_light = False
    green_light = False
    camera_setting = 1500
    CAM_UPPER_LIMIT = 1850
    CAM_LOWER_LIMIT = 1350
    reverse_negative = 1
    prev_start = 0
    last_newton_setting = 0
    last_sketch_newton_setting = 0
    newton_pwm = 1500
    sketch_newton_pwm = 1500

    try:
        controller = XboxController()
        found_controller = True
    except:
        logging.warn("WARNING: No controller is plugged in.")
        time.sleep(3)
        while not found_controller:
            try:
                controller = XboxController()
                found_controller = True
            except:
                logging.warn("No controller found. A controller is required.")
                time.sleep(3)
    
    gyro = []
    thruster_corrections = np.array([0, 0, 0, 0, 0, 0, 0, 0])
    while True:

        networking.scan()

        if networking.accel_data.qsize() > 0:
            accel = networking.accel_data.get_nowait()
        if networking.gyro_data.qsize() > 0:
            gyro.append(networking.gyro_data.get_nowait())

        try:
            inputs = controller.read()
            inputs["lsy"] *= -1
            inputs["lsx"] *= -1
            inputs["rsx"] *= -1
        except:
            logging.error("ERROR: No controller is plugged in.")
            exit(1)
        
        if DEADZONE:
                if abs(inputs["lsy"]) <= DEADZONE_RADIUS:
                    inputs["lsy"] = 0
                if abs(inputs["lsx"]) <= DEADZONE_RADIUS:
                    inputs["lsx"] = 0
                if abs(inputs["rsy"]) <= DEADZONE_RADIUS:
                    inputs["rsy"] = 0
                if abs(inputs["rsx"]) <= DEADZONE_RADIUS:
                    inputs["rsx"] = 0
        if not INVERT_RSY:
                inputs["rsy"] *= -1
        
        if precise_mode:
            inputs["lsy"] *= .25
            inputs["lsx"] *= .25
            inputs["rsy"] *= .25
            inputs["rsx"] *= .5
        if square_inputs:
            inputs["lsy"] *= abs(inputs["lsy"])
            inputs["lsx"] *= abs(inputs["lsx"])
            inputs["rsy"] *= abs(inputs["rsy"]) ** 3
            inputs["rsx"] *= abs(inputs["rsx"])

        thruster_input = np.zeros(6)
        if thrusters_enabled:
            thruster_input[1] = reverse_negative * inputs["lsy"]
            thruster_input[0] = -1 * reverse_negative * inputs["lsx"]
            thruster_input[2] = (inputs["rt"] - inputs["lt"]) * 4 * VERTICAL_SCALE_FACTOR
            thruster_input[3] = inputs["rsx"]
            thruster_input[5] = -1 * reverse_negative * inputs["rsy"] * PITCH_SCALE_FACTOR
            thruster_input[4] = reverse_negative * (inputs["y"] - inputs["x"]) * ROLL_SCALE_FACTOR

        newton_setting = inputs["rb"]
        if not newton_setting and last_newton_setting:
            if newton_pwm == 1800:
                newton_pwm = 1200
            else:
                newton_pwm = 1800
        networking.send(networking.construct_manip_packet(b'N', newton_pwm))
        last_newton_setting = round(newton_setting)

        sketch_newton_setting = inputs["lb"]
        if not sketch_newton_setting and last_sketch_newton_setting:
            if sketch_newton_pwm == 1800:
                sketch_newton_pwm = 1200
            else:
                sketch_newton_pwm = 1800
        networking.send(networking.construct_manip_packet(b'S', sketch_newton_pwm))
        last_sketch_newton_setting = round(sketch_newton_setting)

        camera_input = inputs["dy"]
        if camera_input == 1 and camera_setting < CAM_UPPER_LIMIT - CAM_SPEED:
            camera_setting += CAM_SPEED
            is_cam_moving = True
        elif camera_input == -1 and camera_setting > CAM_LOWER_LIMIT + CAM_SPEED:
            camera_setting -= CAM_SPEED
            is_cam_moving = True
        else:
            is_cam_moving = False
        networking.send(networking.construct_camera_packet(round(camera_setting)))

        if previous_ls and not inputs['ls']:
            precise_mode = not precise_mode
        previous_ls = inputs['ls']

        if prev_start and not inputs["start"]:
            reverse_negative *= -1
        prev_start = inputs["start"]

        if (previous_dx == 1 or previous_dx == -1) and not inputs['dx']:
            if previous_dx + 1:
                green_light = not green_light
                networking.send(b'L' + b'G' + str(int(green_light)).encode(ENCODING))
            else:
                red_light = not red_light
                networking.send(b'L' + b'R' + str(int(green_light)).encode(ENCODING))
        previous_dx = inputs['dx']

        if DO_IMU_HANDLING:

            if DO_GYRO_HANDLING and len(gyro) > 1:
                expected_torques = torques_from_thrusters(thrust_per_thruster).T
                yaw_accel = backwards_difference(gyro[-1].yaw * THRUSTER_RADIUS, gyro[-2].yaw * THRUSTER_RADIUS, gyro[-1].time - gyro[-2].time)
                pitch_accel = backwards_difference(gyro[-1].pitch * THRUSTER_RADIUS, gyro[-2].pitch * THRUSTER_RADIUS, gyro[-1].time - gyro[-2].time)
                roll_accel = backwards_difference(gyro[-1].roll * THRUSTER_RADIUS, gyro[-2].roll * THRUSTER_RADIUS, gyro[-1].time - gyro[-2].time)
                real_torques = (ROBOT_INERTIA @ np.array([yaw_accel, pitch_accel, roll_accel]).T).T
                corrective_torques = expected_torques - real_torques
                pass

            if DO_ACCEL_HANDLING:
                expected_thrusts = thrusts_from_thrusters(thrust_per_thruster).T
                real_thrusts = ROBOT_MASS * np.array([accel.x, accel.y, accel.z])
                corrective_thrusts = expected_thrusts - real_thrusts
                pass

        thruster_percent_vector = thruster_percents(np.array(thruster_input))
        thrust_per_thruster = percent_to_thrust(thruster_percent_vector)

        if DO_IMU_HANDLING:
            CORRECTION_RESERVED_CURRENT = 5
        else: 
            CORRECTION_RESERVED_CURRENT = 0

        if newton_setting != 1500 and is_cam_moving:
            current_max = TOTAL_CURRENT - NEWTON_CURRENT - SERVO_CURRENT - EXTRA_CURRENT - CORRECTION_RESERVED_CURRENT
        elif is_cam_moving:
            current_max = TOTAL_CURRENT - SERVO_CURRENT - EXTRA_CURRENT - CORRECTION_RESERVED_CURRENT
        elif newton_setting != 1500:
            current_max = TOTAL_CURRENT - NEWTON_CURRENT - EXTRA_CURRENT - CORRECTION_RESERVED_CURRENT
        else:
            current_max = TOTAL_CURRENT - EXTRA_CURRENT - CORRECTION_RESERVED_CURRENT
        
        while current_check(thrust_per_thruster) > current_max:
            thrust_per_thruster *= CURRENT_REDUCTION_SCALAR

        if DO_IMU_HANDLING:
            desired_thrusts = thrust_per_thruster + thruster_corrections
        else:
            desired_thrusts = thrust_per_thruster

        thruster_pwm = v_thrust_to_pwm(desired_thrusts)
        networking.send(networking.construct_thruster_packet(thruster_pwm))

        while len(gyro) > 2:
            gyro.pop(0)

        time.sleep(0.01)

def clearPhotos():
    if os.path.exists("C:\\Users\\robosharks\\Documents\\photogrammetry"):
        for file in os.listdir("C:\\Users\\robosharks\\Documents\\photogrammetry"): 
            os.remove(os.path.join("C:\\Users\\robosharks\\Documents\\photogrammetry", file))
    else:
        os.mkdir("C:\\Users\\robosharks\\Documents\\photogrammetry")

if __name__ == "__main__":
    # p1 = Process(target=cam_process, args=(CAM_ONE_ARRAY, CAMERA_ONE_SERIAL))
    # p2 = Process(target=cam_process, args=(CAM_TWO_ARRAY, CAMERA_TWO_SERIAL))

    # p1.start()
    # p2.start()
    clearPhotos()
    main()
