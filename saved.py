# Library imports
from vex import *

# global instances
brain = Brain()
controller = Controller()
brain_inertial = Inertial(Ports.PORT6)


# MotorGroup "left_motors".
left_motor_f = Motor(Ports.PORT10, GearSetting.RATIO_6_1, True)
left_motor_b = Motor(Ports.PORT8, GearSetting.RATIO_6_1, True)
left_motor_t = Motor(Ports.PORT9, GearSetting.RATIO_6_1, False)
left_motors = MotorGroup(left_motor_f, left_motor_b, left_motor_t)

# MotorGroup "right_motors".
right_motor_f = Motor(Ports.PORT13, GearSetting.RATIO_6_1, True)
right_motor_b = Motor(Ports.PORT12, GearSetting.RATIO_6_1, False)
right_motor_t = Motor(Ports.PORT11, GearSetting.RATIO_6_1, True)

right_motors = MotorGroup(right_motor_b, right_motor_b, right_motor_t)

# smartdrive
# smartdrive = SmartDrive(left_motors, right_motors, brain_inertial, wheelTravel, trackWidth, wheelBase, INCHES, externalGearRatio)
smartdrive = SmartDrive(left_motors, right_motors, brain_inertial, 156, 340, 230, MM, 0.6)


# Clamp and Scoring mech
clamp_pneumatic = DigitalOut(brain.three_wire_port.h)
intake_motor = Motor(Ports.PORT7, GearSetting.RATIO_6_1, True)
belt_motor = Motor(Ports.PORT14, GearSetting.RATIO_6_1, True)

#global vars
is_intake_spinning = False
is_belt_spinning = False


def set_initial_params():
    """ Set initial params
    """
    brain_inertial.set_heading(90)
    # P - Gain. Smartdrive uses simple P controller
    # set Kp for P controller
    smartdrive.set_turn_constant(0.5)
    # set P controller exit error marging
    smartdrive.set_turn_threshold(2)


def calibrate_inertial():
    """ Calibrate Inertials sensor
    """
    # Start calibration.
    brain_inertial.calibrate()
    # Print that the Inertial Sensor is calibrating while
    # waiting for it to finish calibrating.
    while brain_inertial.is_calibrating():
        brain.screen.clear_line(2)
        brain.screen.set_cursor(2,2)
        brain.screen.print("Inertial Sensor Calibrating")
        wait(50, MSEC)
    brain.screen.clear_line(2)


def show_me_info():
    """ Debug Thread
    """
    while True:
        brain.screen.set_cursor(2, 2)
        brain.screen.print("Right Motors: " + str(right_motors.position()))
        
        brain.screen.set_cursor(3, 2)
        brain.screen.print("Left Motors: " + str(left_motors.position()))

        brain.screen.set_cursor(4, 2)
        brain.screen.print("Heading: " + str(brain_inertial.heading())) # type: ignore

        wait(100, MSEC)
        brain.screen.clear_screen()

def clamp():
    """ Pneumatic clamp function
    """
    if not clamp_pneumatic.value():
        clamp_pneumatic.set(True)
    else:
        clamp_pneumatic.set(False) 

def intake(dir = FORWARD, spd = 90):
    """ Start and Stop Intake
    """
    global is_intake_spinning
    if not is_intake_spinning:
        intake_motor.spin(dir, spd, VelocityUnits.PERCENT)
        is_intake_spinning = True
    else:
        intake_motor.stop()
        is_intake_spinning = False     
    

def belt(dir = FORWARD, spd = 90):
    """ Start Belt
    """
    global is_belt_spinning
    if not is_belt_spinning:
        belt_motor.spin(dir, spd, VelocityUnits.PERCENT)
        is_belt_spinning = True
    else:
        belt_motor.stop()
        is_belt_spinning = False     


def score():
    """ Start Intake and Belt
    """
    intake()
    belt()

def autonomous():
    """ Add autonomous code
    """
    pass

def user_control():
    brain.screen.clear_screen()
    brain.screen.print("driver control")
    # driver joystick control
    while True:

        throttle = controller.axis3.position()
        turn = controller.axis1.position()
        left_motors.spin(FORWARD, throttle + turn,VelocityUnits.PERCENT)
        right_motors.spin(FORWARD, throttle - turn ,VelocityUnits.PERCENT)

        wait(20, MSEC)

# create competition instance
comp = Competition(user_control, autonomous)

# actions to do when the program starts
brain.screen.clear_screen()

# bind button callback functions
controller.buttonL1.pressed(clamp)
controller.buttonR1.pressed(score)

# set params, calibrate inertial
calibrate_inertial()
set_initial_params()

# start thread to output data for debug
thread = Thread(show_me_info)

"""
class AdvancedAddressableLed(AddressableLed):
    pass

    def all_red(self):
        self.set([Color(0x800000)] * 60)

    def all_green(self):
        self.set([Color(0x008000)] * 60)

    def all_blue(self):
        self.set([Color(0x1414e5)] * 60)

    def rotate(self, data):
        data[:] = data[1:] + data[:1]
        self.set(data)

led=AdvancedAddressableLed(brain.three_wire_port.b)

# 60 led colors

c=[Color(127,25,25),
   Color(127,76,25),
   Color(127,176,45),
   Color(173,176,45),
   Color(40,76,165),
   Color(176,72,72)
  ] * 10

def led_run():
    led.clear()
    sleep(1500)
    led.all_blue()
    sleep(1500)
    led.all_red()
    sleep(1500)
    led.all_green()
    sleep(1500)
    led.clear()
    sleep(500)

    colors = []
    for n in range(60):
        colors.append(Color(176,72,72))
        led.set(colors, 0)
        sleep(50)

    led.clear()
    sleep(500)

    while True:
        led.rotate(c)
        sleep(500)

led_run()


"""

