# Library imports
from vex import *


# PID Controller class
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
        """
            Calculate PID controller output
        """
        self.error = error
        # Proportional component
        P = self.error * self.kp

        # Prevent integral windup. If error is less than start_integrate, start accumalating integral
        if abs(self.error) < self.start_integrate:
            self.integral = self.integral + self.error
        else: 
            self.integral = 0

        # if error crosses 0 and changes sign, reset accumulated integral
        if (self.prev_error < 0 and self.error > 0) or (self.prev_error > 0 and self.error < 0):
            self.integral = 0

        # calculate integral component
        I = + self.integral * self.ki
        # calculate derivative
        D = (self.error - self.prev_error) * self.kd
        # total gain
        total = P + I + D
        self.prev_error = self.error
        # clamp and return
        return self.max * ( abs(P) / P ) if abs(total) > self.max else total
    
    def is_ready(self):
        """
            returns true if PID is settled
        """
        if abs(self.error) < self.exit_error:
            return True
        return False

# Robotclass forDrive & Turn control
class Robot:
    def __init__(self,l_mg: MotorGroup, r_mg: MotorGroup, imu: Inertial, wheel_diameter: float, gear_ratio: float):
        self.inertial = imu
        self.left_motors = l_mg
        self.right_motors = r_mg
        self.wheel_diameter = wheel_diameter
        self.gear_ratio = gear_ratio
        self.drive_def_kp = 0
        self.drive_def_ki = 0
        self.drive_def_kd = 0
        self.drive_def_si = 0
        self.drive_def_exit = 0

    def set_drive_defaults(self, kp, ki, kd, start_integrate, exit_error):
        """
            set default PID parameters for drive
        """
        self.drive_def_kp = kp
        self.drive_def_ki = ki
        self.drive_def_kd = kd
        self.drive_def_si = start_integrate
        self.drive_def_exit = exit_error

    def set_turn_defaults(self, kp, ki, kd, start_integrate, exit_error, max_speed):
        """
            set default PID parameters for turn
        """        
        self.turn_def_kp = kp
        self.turn_def_ki = ki
        self.turn_def_kd = kd
        self.turn_def_si = start_integrate
        self.turn_def_exit = exit_error

    @staticmethod
    def angle_optimizer(angle: float):
        """
            if robot requested to turn more than 180 degress, then it turns to opposite side to save time
        """
        angle = angle % 360
        if angle < -180:
            angle = angle + 360
        elif angle > 180:
            angle = angle - 360
        return angle

    def reset_encoders(self):
        """
            Resets motor encoders
        """
        self.left_motors.reset_position()
        self.right_motors.reset_position()

    def get_encoder_distance(self):
        """
            return encoder distance based on average motor group left and right, wheel diameter and gear set
        """
        return ((self.left_motors.position(TURNS) + self.right_motors.position(TURNS)) / 2) * (self.wheel_diameter * 3.14) * self.gear_ratio

    def drive_for(self, distance, max_speed = 8, heading = -1):
        """
            Robot Drives to specified distance with specified max_speed. If heading is not specified it drives staight
        """       
        if heading == -1:
            heading = self.inertial.heading()

        self.reset_encoders()

        drive_pid = PID(distance, self.drive_def_kp, self.drive_def_ki, self.drive_def_kd , self.drive_def_si , self.drive_def_exit, max_speed)
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
            self.left_motors.spin(FORWARD, drive_speed * 8.3 - heading_correction_speed  , VelocityUnits.PERCENT)
            self.right_motors.spin(FORWARD, drive_speed * 8.3 + heading_correction_speed , VelocityUnits.PERCENT)
            wait(20, MSEC)

        self.left_motors.spin(FORWARD, 0)
        self.right_motors.spin(FORWARD, 0)        

    def turn_to(self, heading):
        pass

# Define all required devices
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

# Clamp and Scoring mech
clamp_pneumatic = DigitalOut(brain.three_wire_port.h)
intake_motor = Motor(Ports.PORT7, GearSetting.RATIO_6_1, True)
belt_motor = Motor(Ports.PORT14, GearSetting.RATIO_6_1, True)

#global variables
is_intake_spinning = False
is_belt_spinning = False

# create my_robot object for autonomous
my_robot = Robot(left_motors, right_motors, brain_inertial, 3.25, 0.6)


def set_initial_params():
    """ Set initial params
    """
    brain_inertial.set_heading(90)
    # set Kp, Ki, Kd, start integrate, exit error, max_speed
    my_robot.set_drive_defaults(1.5, 0, 8, 1, 1)


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
    """ 
        Starts and Stops Intake
    """
    global is_intake_spinning
    if not is_intake_spinning:
        intake_motor.spin(dir, spd, VelocityUnits.PERCENT)
        is_intake_spinning = True
    else:
        intake_motor.stop()
        is_intake_spinning = False     
    

def belt(dir = FORWARD, spd = 90):
    """ 
        Starts and Stops Belt
    """
    global is_belt_spinning
    if not is_belt_spinning:
        belt_motor.spin(dir, spd, VelocityUnits.PERCENT)
        is_belt_spinning = True
    else:
        belt_motor.stop()
        is_belt_spinning = False     


def score():
    """ 
        Button callback. Starts Intake and Belt
    """
    intake()
    belt()

def autonomous():
    """
        Autonomous Code
    """
    my_robot.drive_for(25)
    wait(1, SECONDS)
    my_robot.drive_for(-25)

def user_control():
    """
        Driver Control Code
    """    
    while True:
        throttle = controller.axis3.position()
        turn = controller.axis1.position()
        left_motors.spin(FORWARD, throttle + turn,VelocityUnits.PERCENT)
        right_motors.spin(FORWARD, throttle - turn ,VelocityUnits.PERCENT)
        wait(20, MSEC)

# create competition instance
comp = Competition(user_control, autonomous)

# Clear Screen
brain.screen.clear_screen()

# bind button callback functions
controller.buttonL1.pressed(clamp)
controller.buttonR1.pressed(score)

# set PID params, calibrate inertial
calibrate_inertial()
set_initial_params()

# start parallel thread for info output
thread = Thread(show_me_info)


