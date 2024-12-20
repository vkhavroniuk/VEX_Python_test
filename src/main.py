# Library imports
from vex import *
# import robot_tools

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

right_motors = MotorGroup(right_motor_b, right_motor_b,right_motor_t)

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


class PID:
    def __init__(self, initial_eror, kp, ki, kd, start_integrate , exit_error, max_speed):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.start_integrate = start_integrate
        self.max = max_speed
        self.exit_error = exit_error
        self.prev_error = initial_eror
        self.error = initial_eror
        self.integral = 0
    
    def calculate(self, error):
        self.error = error
        P = self.error * self.kp
        if abs(self.error) < self.start_integrate:
            self.integral = self.integral + self.error
        else: 
            self.integral = 0
        I = + self.integral * self.ki
        D = (self.error - self.prev_error) * self.kd
        total = P + I + D
        self.prev_error = self.error
        return self.max * ( abs(P) / P ) if abs(total) > self.max else total
    
    def is_ready(self):
        if abs(self.error) < self.exit_error:
            return True
        return False

# class forDrive & Turn
class Robot:
    def __init__(self,l_mg: MotorGroup, r_mg: MotorGroup, imu: Inertial, wheel_diameter: float, gear_ratio: float):
        self.inertial = imu
        self.left_motors = l_mg
        self.right_motors = r_mg
        self.wheel_diameter = wheel_diameter
        self.gear_ratio = gear_ratio

    @staticmethod
    def angle_optimizer(angle: float):
        angle = angle % 360
        if angle < -180:
            angle = angle + 360
        elif angle > 180:
            angle = angle - 360
        return angle

    def reset_encoders(self):
        self.left_motors.reset_position()
        self.right_motors.reset_position()

    def get_encoder_distance(self):
        """
            return encoder distance based on average motor group left and right, wheel diameter and gear set
        """
        return ((self.left_motors.position(TURNS) + self.right_motors.position(TURNS)) / 2) * (self.wheel_diameter * 3.14) * self.gear_ratio

    def drive_for(self, distance, max_power = 8, heading = -1):
        if heading == -1:
            heading = self.inertial.heading()
        # self.left_motors.reset_position()
        # self.left_motors.reset_position()
        self.reset_encoders()

        drive_pid = PID(distance, 1.5, 0, max_power ,1 , 1, 6)
        steer_pid = PID(0, 0.4, 0, 1, 0 , 1, 6)

        while not drive_pid.is_ready():
            # request encoder position
            position = self.get_encoder_distance()
            # calculate ramaining error
            drive_error = distance - position
            # calculate heading correction error
            heading_error = heading - self.inertial.heading()
            # get PID feedback for heading and drive
            heading_correction_speed = steer_pid.calculate(self.angle_optimizer(heading_error))
            drive_speed = drive_pid.calculate(drive_error)
            
            # temporary debug output
            print("Pos:", position)
            print("Error:", drive_error)
            print("Drive Speed:", drive_speed)
            print("Heading Error:", heading_error)
            print("###################################")
            # end of temporary debug output

            # set motor speed
            self.left_motors.spin(FORWARD, drive_speed * 8.3  , VelocityUnits.PERCENT)
            self.right_motors.spin(FORWARD, drive_speed * 8.3 , VelocityUnits.PERCENT)
            wait(20, MSEC)

        self.left_motors.spin(FORWARD, 0)
        self.right_motors.spin(FORWARD, 0)        

my_robot = Robot(left_motors, right_motors, brain_inertial, 3.25, 0.6)

def set_initial_params():
    """ Set initial params
    """
    brain_inertial.set_heading(90)
    # P - Gain. Smartdrive uses simple P controller
    smartdrive.set_turn_constant(0.5)
    # Exit Error
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

    my_robot.drive_for(25)
    wait(1, SECONDS)
    my_robot.drive_for(-25)

def user_control():
    brain.screen.clear_screen()
    brain.screen.print("driver control")
    # place driver control in this while loop
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

# bind buttons
controller.buttonL1.pressed(clamp)
controller.buttonR1.pressed(score)

# set params, calibrate inertial
calibrate_inertial()
set_initial_params()

thread = Thread(show_me_info)


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



