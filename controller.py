# code for controller input class
from inputs import get_gamepad
import math
import threading

# class to retrieve Xbox controller input
class XboxController(object):
    # constants for normalizing input
    MAX_TRIG_VAL = math.pow(2, 10)
    MAX_JOY_VAL = math.pow(2, 15)

    def __init__(self):

        # set up all the vars needed to store input
        self.LeftJoystickY = 0
        self.LeftJoystickX = 0
        self.RightJoystickY = 0
        self.RightJoystickX = 0
        self.LeftTrigger = 0
        self.RightTrigger = 0
        self.LeftBumper = 0
        self.RightBumper = 0
        self.A = 0
        self.X = 0
        self.Y = 0
        self.B = 0
        self.LeftThumb = 0
        self.RightThumb = 0
        self.Back = 0
        self.Start = 0
        self.DPadX = 0
        self.DPadY = 0

        # begin monitoring input
        self._monitor_thread = threading.Thread(target=self._monitor_controller, args=())
        self._monitor_thread.daemon = True
        self._monitor_thread.start()


    def read(self): 
        # return the button/trigger states
        return {'a': self.A, 
                'b': self.B,
                'x': self.X,
                'y': self.Y,
                'ls': self.LeftThumb,
                'rs': self.RightThumb,
                'back': self.Back,
                'start': self.Start,
                'lb': self.LeftBumper,
                'rb': self.RightBumper,
                'dx': self.DPadX,
                'dy': self.DPadY,
                'lt': self.LeftTrigger,
                'rt': self.RightTrigger,
                'lsx': self.LeftJoystickX,
                'lsy': self.LeftJoystickY,
                'rsx': self.RightJoystickX,
                'rsy': self.RightJoystickY}


    def _monitor_controller(self):
        while True:
            events = get_gamepad()

            # use event codes to set input values
            for event in events:
                #print(event.code + ": " + str(event.state)) # print code/value for debugging
                if event.code == 'ABS_Y':
                    self.LeftJoystickY = event.state / XboxController.MAX_JOY_VAL # normalize between -1 and 1
                elif event.code == 'ABS_X':
                    self.LeftJoystickX = event.state / XboxController.MAX_JOY_VAL # normalize between -1 and 1
                elif event.code == 'ABS_RY':
                    self.RightJoystickY = event.state / XboxController.MAX_JOY_VAL # normalize between -1 and 1
                elif event.code == 'ABS_RX':
                    self.RightJoystickX = event.state / XboxController.MAX_JOY_VAL # normalize between -1 and 1
                elif event.code == 'ABS_Z':
                    self.LeftTrigger = event.state / XboxController.MAX_TRIG_VAL # normalize between 0 and 1
                elif event.code == 'ABS_RZ':
                    self.RightTrigger = event.state / XboxController.MAX_TRIG_VAL # normalize between 0 and 1
                elif event.code == 'BTN_TL':
                    self.LeftBumper = event.state
                elif event.code == 'BTN_TR':
                    self.RightBumper = event.state
                elif event.code == 'BTN_SOUTH':
                    self.A = event.state
                elif event.code == 'BTN_NORTH':
                    self.X = event.state
                elif event.code == 'BTN_WEST':
                    self.Y = event.state
                elif event.code == 'BTN_EAST':
                    self.B = event.state
                elif event.code == 'BTN_THUMBL':
                    self.LeftThumb = event.state
                elif event.code == 'BTN_THUMBR':
                    self.RightThumb = event.state
                elif event.code == 'BTN_SELECT':
                    self.Back = event.state
                elif event.code == 'BTN_START':
                    self.Start = event.state
                elif event.code == 'ABS_HAT0X':
                    self.DPadX = event.state
                elif event.code == 'ABS_HAT0Y':
                    self.DPadY = event.state

if __name__ == '__main__':
    import time
    controller = XboxController()
    while True:
        print(controller.read())
        time.sleep(3)
