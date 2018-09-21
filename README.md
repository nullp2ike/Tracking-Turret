# Tracking-Turret
A motion tracking turret for https://www.youtube.com/watch?v=HoRPWUl_sF8

## Install Guide

Make sure pip is installed. 
```bash
sudo apt-get install python pip
```

Setup I2C on your Raspberry Pi

https://learn.adafruit.com/adafruits-raspberry-pi-lesson-4-gpio-setup/configuring-i2c

Install the Adafruit PCA9685 PWM servo library.

https://github.com/adafruit/Adafruit_Python_PCA9685

```bash
git clone https://github.com/adafruit/Adafruit_Python_PCA9685.git
cd Adafruit_Python_PCA9685
sudo python setup.py install
sudo pip install adafruit-pca9685
```

Install OpenCV 3. Follow all steps for python 2.7 instructions

http://www.pyimagesearch.com/2016/04/18/install-guide-raspberry-pi-3-raspbian-jessie-opencv-3/

Make sure to create your virtual environment with the extra flag.

```bash
mkvirtualenv cv --system-site-packages -p python2
```

Source your bash profile

```bash
source ~/.profile
```

Activate your virtual environment

```
workon cv
```

Clone this repository

```
git clone git@github.com:nullp2ike/Tracking-Turret.git
```

Navigate to the directory

```
cd Tracking-Turret
```

Install dependencies to your virtual environment

```
pip install imutils RPi.GPIO
```

Run the project!

```
python turret.py
```

## Command-Line Arguments

```
usage: turret.py [-h] [-m {1,2}] [-v {y,n}]

optional arguments:
  -h, --help            show this help message and exit
  -m {1,2}, --mode {1,2}
                        Input mode. (1) Motion Detection, (2) Interactive
  -v {y,n}, --video {y,n}
                        Live video (y, n)
```

## Setting Parameters

turret.py has a couple parameters that you can set.
Look for the `User Parameters` section.

```python
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

LOG_MOVEMENT = True         # Log movement
FRIENDLY_MODE = True        # Friendly mode (allow firing for Motion Detection)

MAX_STEPS_X = 50            # Max motion tracking X-axis movement
MAX_STEPS_Y = 14            # Max motion tracking Y-axis movement

#######################
```

