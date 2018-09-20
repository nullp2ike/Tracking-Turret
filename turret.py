try:
    import cv2
except Exception as e:
    print("Warning: OpenCV not installed. To use motion detection, make sure you've properly configured OpenCV.")

import time
import thread
import threading
import atexit
import sys
import termios
import contextlib
import argparse

import imutils
import RPi.GPIO as GPIO
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor, Adafruit_StepperMotor
import Adafruit_PCA9685

### User Parameters ###

SERVO_MIN = 1000            # Servo min pulse
SERVO_MAX = 2000            # Servo max pulse

MOTOR_X_STARTPOS = 1500     # X-axis (base) servo start position
MOTOR_Y_STARTPOS = 1050     # Y-axis (gun) servo start position
MOTOR_FIRE_STARTPOS = 1150  # Start position of the bullet pushing motor
MOTOR_FIRE_ENDPOS = 1850    # End position of the bullet pushing motor

TURRET_SETUP_TIME = 3       # Load time in seconds
MAX_NR_OF_ROUNDS = 6        # Ammo storage capacity

MANUAL_MOVE_STEP = 5        # Movement modifier for Interactive mode
VIDEO_MOVE_STEP = 2         # Movement modifier for Motion Detection mode
MOVEMENT_MODIFIER = 10      # Multiply movement step by this modifier

RELAY_PIN = 22              # Relay PIN number on the board

LOG_MOVEMENT = False        # Log movement
FRIENDLY_MODE = True        # Friendly mode (allow firing for Motion Detection)

MAX_STEPS_X = (SERVO_MAX - SERVO_MIN) / (VIDEO_MOVE_STEP * MOVEMENT_MODIFIER)
MAX_STEPS_Y = (SERVO_MAX - SERVO_MIN) / (VIDEO_MOVE_STEP * MOVEMENT_MODIFIER)

#######################


@contextlib.contextmanager
def raw_mode(file):
    """
    Magic function that allows key presses.
    :param file:
    :return:
    """
    old_attrs = termios.tcgetattr(file.fileno())
    new_attrs = old_attrs[:]
    new_attrs[3] = new_attrs[3] & ~(termios.ECHO | termios.ICANON)
    try:
        termios.tcsetattr(file.fileno(), termios.TCSADRAIN, new_attrs)
        yield
    finally:
        termios.tcsetattr(file.fileno(), termios.TCSADRAIN, old_attrs)


class VideoUtils(object):
    """
    Helper functions for video utilities.
    """
    @staticmethod
    def live_video(camera_port=0):
        """
        Opens a window with live video.
        :param camera:
        :return:
        """

        video_capture = cv2.VideoCapture(camera_port)

        while True:
            # Capture frame-by-frame
            ret, frame = video_capture.read()

            # Display the resulting frame
            cv2.imshow('Video', frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        # When everything is done, release the capture
        video_capture.release()
        cv2.destroyAllWindows()

    @staticmethod
    def find_motion(callback, camera_port=0, show_video=False):

        camera = cv2.VideoCapture(camera_port)
        time.sleep(0.25)

        # initialize the first frame in the video stream
        firstFrame = None
        tempFrame = None
        count = 0

        # loop over the frames of the video
        while True:
            # grab the current frame and initialize the occupied/unoccupied
            # text

            (grabbed, frame) = camera.read()

            # if the frame could not be grabbed, then we have reached the end
            # of the video
            if not grabbed:
                break

            # resize the frame, convert it to grayscale, and blur it
            frame = imutils.resize(frame, width=500)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            gray = cv2.GaussianBlur(gray, (21, 21), 0)

            # if the first frame is None, initialize it
            if firstFrame is None:
                print "Waiting for video to adjust..."
                if tempFrame is None:
                    tempFrame = gray
                    continue
                else:
                    delta = cv2.absdiff(tempFrame, gray)
                    tempFrame = gray
                    tst = cv2.threshold(delta, 5, 255, cv2.THRESH_BINARY)[1]
                    tst = cv2.dilate(tst, None, iterations=2)
                    if count > 30:
                        print "Done.\n Waiting for motion."
                        if not cv2.countNonZero(tst) > 0:
                            firstFrame = gray
                        else:
                            continue
                    else:
                        count += 1
                        continue

            # compute the absolute difference between the current frame and
            # first frame
            frameDelta = cv2.absdiff(firstFrame, gray)
            thresh = cv2.threshold(frameDelta, 25, 255, cv2.THRESH_BINARY)[1]

            # dilate the thresholded image to fill in holes, then find contours
            # on thresholded image
            thresh = cv2.dilate(thresh, None, iterations=2)
            c = VideoUtils.get_best_contour(thresh.copy(), 5000)

            if c is not None:
                # compute the bounding box for the contour, draw it on the frame,
                # and update the text
                (x, y, w, h) = cv2.boundingRect(c)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                callback(c, frame)

            # show the frame and record if the user presses a key
            if show_video:
                cv2.imshow("Security Feed", frame)
                key = cv2.waitKey(1) & 0xFF

                # if the `q` key is pressed, break from the lop
                if key == ord("q"):
                    break

        # cleanup the camera and close any open windows
        camera.release()
        cv2.destroyAllWindows()

    @staticmethod
    def get_best_contour(imgmask, threshold):
        im, contours, hierarchy = cv2.findContours(imgmask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best_area = threshold
        best_cnt = None
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > best_area:
                best_area = area
                best_cnt = cnt
        return best_cnt


class Turret(object):
    """
    Class used for turret control.
    """
    def __init__(self, friendly_mode=True):
        self.friendly_mode = friendly_mode

        # create a default object, no changes to I2C address or frequency
        #self.mh = Adafruit_MotorHAT()
        self.pwm = Adafruit_PCA9685.PCA9685()
        atexit.register(self.__turn_off_motors)

        # Init servos
        self.pwm.set_pwm_freq(60)

        # Relay
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(RELAY_PIN, GPIO.OUT)
        GPIO.output(RELAY_PIN, GPIO.HIGH)

        # Servo channels
        self.sm_x = 0
        self.sm_y = 1
        self.sm_fire = 2

        # Ammo count
        self.ammo_left = MAX_NR_OF_ROUNDS

        self.__init_motor_positions()

    # Helper function to make setting a servo pulse width simpler.
    def set_servo_pulse(self, channel, pulse, log=LOG_MOVEMENT):
        #print('channel: {0}, pulse: {1}'.format(channel, pulse))
        pulse_length = 1000000    # 1,000,000 us per second
        pulse_length //= 60       # 60 Hz
        #print('{0}us per period'.format(pulse_length))
        pulse_length //= 4096     # 12 bits of resolution
        #print('{0}us per bit'.format(pulse_length))
        #pulse *= 1000
        pulse //= pulse_length
        if log:
            print('channel: {0}, pulse final: {1}\n'.format(channel, pulse))
        self.pwm.set_pwm(channel, 0, pulse)

    def motion_detection(self, show_video=False):
        """
        Uses the camera to move the turret. OpenCV ust be configured to use this.
        :return:
        """
        VideoUtils.find_motion(self.__move_axis, show_video=show_video)

    def __move_axis(self, contour, frame):
        (v_h, v_w) = frame.shape[:2]
        (x, y, w, h) = cv2.boundingRect(contour)

        # find height
        target_steps_x = (2*MAX_STEPS_X * (x + w / 2) / v_w) - MAX_STEPS_X
        target_steps_y = (2*MAX_STEPS_Y * (y + h / 2) / v_h) - MAX_STEPS_Y

        print "x: %s, y: %s" % (str(target_steps_x), str(target_steps_y))
        print "current x: %s, current y: %s" % (str(self.current_x_steps), str(self.current_y_steps))

        t_x = threading.Thread()
        t_y = threading.Thread()
        t_fire = threading.Thread()

        # move x
        if (target_steps_x - self.current_x_steps) > 0:
            self.current_x_steps += 1
            t_x = threading.Thread(target=self.move_backward, args=(self.sm_x, VIDEO_MOVE_STEP,))
        elif (target_steps_x - self.current_x_steps) < 0:
            self.current_x_steps -= 1
            t_x = threading.Thread(target=self.move_forward, args=(self.sm_x, VIDEO_MOVE_STEP,))

        # move y
        if (target_steps_y - self.current_y_steps) > 0:
            self.current_y_steps += 1
            t_y = threading.Thread(target=self.move_forward, args=(self.sm_y, VIDEO_MOVE_STEP,))
        elif (target_steps_y - self.current_y_steps) < 0:
            self.current_y_steps -= 1
            t_y = threading.Thread(target=self.move_backward, args=(self.sm_y, VIDEO_MOVE_STEP,))

        # fire if necessary
        if not self.friendly_mode:
            if abs(target_steps_y - self.current_y_steps) <= VIDEO_MOVE_STEP and abs(target_steps_x - self.current_x_steps) <= VIDEO_MOVE_STEP:
                t_fire = threading.Thread(target=self.fire)

        t_x.start()
        t_y.start()
        t_fire.start()

        t_x.join()
        t_y.join()
        t_fire.join()

    def interactive(self):
        """
        Starts an interactive session. Key presses determine movement.
        :return:
        """

        print 'Commands:'
        print '(WASD) to move'
        print '(ENTER) to fire'
        print '(c) for continuous fire'
        print '(i) and (o) to manually turn on gun motor'
        print 'Exit with (q)\n'
        with raw_mode(sys.stdin):
            try:
                while True:
                    ch = sys.stdin.read(1)
                    if not ch or ch == "q":
                        break

                    if ch == "w":
                        self.move_backward(self.sm_y, MANUAL_MOVE_STEP)
                    elif ch == "s":
                        self.move_forward(self.sm_y, MANUAL_MOVE_STEP)
                    elif ch == "a":
                        self.move_forward(self.sm_x, MANUAL_MOVE_STEP)
                    elif ch == "d":
                        self.move_backward(self.sm_x, MANUAL_MOVE_STEP)
                    elif ch == "\n":
                        if not self.MANUAL_TURRET_ON:
                            self.turret_on()

                        self.fire()

                        if not self.MANUAL_TURRET_ON:
                            self.turret_off()
                    elif ch == "c":
                        self.turret_on()
                        fire_count = 0
                        while fire_count < MAX_NR_OF_ROUNDS:
                            self.fire()
                            fire_count += 1
                            time.sleep(1)
                        self.turret_off()
                    elif ch == "i":
                        self.MANUAL_TURRET_ON = True
                        self.turret_on(False)
                    elif ch == "o":
                        self.MANUAL_TURRET_ON = False
                        self.turret_off()

            except (KeyboardInterrupt, EOFError):
                pass

    def turret_on(self, sleep=True):
        if self.ammo_left > 0:
            GPIO.output(RELAY_PIN, GPIO.LOW)
            if sleep:
                time.sleep(TURRET_SETUP_TIME)
        else:
            sys.exit("OUT OF AMMO")

    def turret_off(self):
        GPIO.output(RELAY_PIN, GPIO.HIGH)

    def fire(self):
        if self.ammo_left > 0:
            print("FIRE!")
            self.ammo_left -= 1
            self.set_servo_pulse(self.sm_fire, MOTOR_FIRE_ENDPOS)
            time.sleep(1)
            self.set_servo_pulse(self.sm_fire, MOTOR_FIRE_STARTPOS)
            time.sleep(1)

            if self.ammo_left == 0:
                sys.exit("OUT OF AMMO")
        else:
            sys.exit("OUT OF AMMO")

    def move_forward(self, motor, steps):
        """
        Moves the stepper motor forward the specified number of steps.
        :param motor:
        :param steps:
        :return:
        """
        pos = self.pos[motor] + (steps * MOVEMENT_MODIFIER)
        if(pos <= SERVO_MAX):
            self.pos[motor] = pos
            self.set_servo_pulse(motor, pos)
        
    def move_backward(self, motor, steps):
        """
        Moves the stepper motor backward the specified number of steps
        :param motor:
        :param steps:
        :return:
        """
        pos = self.pos[motor] - (steps * MOVEMENT_MODIFIER)
        if(pos >= SERVO_MIN):
            self.pos[motor] = pos
            self.set_servo_pulse(motor, pos)

    def __init_motor_positions(self):
        self.pos = [MOTOR_X_STARTPOS, MOTOR_Y_STARTPOS, MOTOR_FIRE_STARTPOS]

        self.set_servo_pulse(self.sm_x, self.pos[self.sm_x], False)
        self.set_servo_pulse(self.sm_y, self.pos[self.sm_y], False)
        self.set_servo_pulse(self.sm_fire, self.pos[self.sm_fire], False)
        
        self.current_x_steps = 0
        self.current_y_steps = 0

        self.MANUAL_TURRET_ON = False

    def __turn_off_motors(self):
        """
        Recommended for auto-disabling motors on shutdown!
        :return:
        """
        GPIO.output(RELAY_PIN, GPIO.HIGH)
        self.__init_motor_positions()

def parseArguments():
    # Create argument parser
    parser = argparse.ArgumentParser()

    # Positional mandatory arguments
    parser.add_argument("-m", "--mode", help="Input mode. (1) Motion Detection, (2) Interactive", type=str, required=False, choices=["1", "2"])
    parser.add_argument("-v", "--video", help="Live video (y, n)", type=str, required=False, choices=["y", "Y", "n"], default="n")

    # Parse arguments
    args = parser.parse_args()

    return args

if __name__ == "__main__":
    t = Turret(friendly_mode=FRIENDLY_MODE)

    args = parseArguments()

    if args.mode:
        # Allow video
        show_video_enabled = False
        if args.video == "y":
            show_video_enabled = True
            
        if args.mode == "1":
            if show_video_enabled:
                print "\nInitializing Motion Detection mode with live video enabled.\n"
                t.motion_detection(show_video=True)
            else:
                print "\nInitializing Motion Detection mode without live video.\n"
                t.motion_detection()
        elif args.mode == "2":
            if show_video_enabled:
                print "\nInitializing Interactive mode with live video enabled.\n"
                thread.start_new_thread(VideoUtils.live_video, ())
            else:
                print "\nInitializing Interactive mode withoput live video.\n"
            t.interactive()
        else:
            print "Invalid mode argument. Available choices: (1) Motion Detection, (2) Interactive\n"
    else: 
        user_input = raw_input("Choose an input mode: (1) Motion Detection, (2) Interactive\n")

        if user_input == "1":
            if raw_input("Live video? (y, n)\n").lower() == "y":
                t.motion_detection(show_video=True)
            else:
                t.motion_detection()
        elif user_input == "2":
            if raw_input("Live video? (y, n)\n").lower() == "y":
                thread.start_new_thread(VideoUtils.live_video, ())
            t.interactive()
        else:
            print "Unknown input mode. Please choose a number (1) or (2)"
