from ctre import CANTalon
import math


class SwerveModule:

    CIMCODER_COUNTS_PER_REV: int = 80
    WHEEL_DIAMETER: float = 0.0254 * 3
    DRIVE_ENCODER_GEAR_REDUCTION: float = 5.43956
    # The VEX Integrated encoders have 1 count per revolution, and there
    # is a 1:1 corrospondence to the angular position of the module.
    STEER_COUNTS_PER_RADIAN = 1.0 / math.tau

    def __init__(self, steer_talon: CANTalon, drive_talon: CANTalon,
            steer_enc_offset: float, x_pos: float, y_pos: float,
            drive_free_speed: float,
            reverse_steer_direction: bool=True,
            reverse_steer_encoder: bool=True,
            reverse_drive_direction: bool=False,
            reverse_drive_encoder: bool=False):

        self.steer_motor = steer_talon
        self.drive_motor = drive_talon
        self.x_pos = x_pos
        self.y_pos = y_pos
        self.steer_enc_offset = steer_enc_offset
        self.reverse_steer_direction = reverse_steer_direction
        self.reverse_steer_encoder = reverse_steer_encoder
        self.reverse_drive_direction = reverse_drive_direction
        self.reverse_drive_encoder = reverse_drive_encoder
        self.drive_free_speed = drive_free_speed

        self.absolute_rotation = True
        self.vx = 0
        self.vy = 0

        self.steer_motor.setControlMode(CANTalon.ControlMode.Position)
        self.steer_motor.setFeedbackDevice(CANTalon.FeedbackDevice.CtreMagEncoder_Absolute)
        # changes sign of motor throttle vilues
        self.steer_motor.reverseOutput(self.reverse_steer_direction)
        # changes direction of motor encoder
        self.steer_motor.reverseSensor(self.reverse_steer_encoder)
        self.steer_motor.setPID(1.0, 0.0, 0.0)
        # Initialise the motor setpoint to its current position. This is to
        # prevent the module unwinding (e.g. if it is wound up to an encoder
        # position of 50, we set the initial setpoint to 50)
        self.steer_motor.set(self.steer_motor.getPosition())

        self.drive_motor.setControlMode(CANTalon.ControlMode.Speed)
        self.drive_motor.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder)
        # changes sign of motor throttle values
        self.drive_motor.reverseOutput(self.reverse_drive_direction)
        # changes direction of motor encoder
        self.drive_motor.reverseSensor(self.reverse_drive_encoder)
        self.drive_motor.setPID(1.0, 0.0, 1024.0/self.drive_free_speed)

        self.drive_counts_per_rev = \
            SwerveModule.CIMCODER_COUNTS_PER_REV*self.DRIVE_ENCODER_GEAR_REDUCTION
        self.drive_counts_per_meter = \
            self.drive_counts_per_rev / (math.pi * self.WHEEL_DIAMETER)

        # factor by which to scale velocities in m/s to give to our drive talon.
        # 0.1 is because SRX velocities are measured in ticks/100ms
        self.drive_velocity_to_native_units = self.drive_counts_per_meter*0.1

    def set_rotation_mode(self, rotation_mode):
        """Set whether we want the modules to rotate to the nearest possible
        direction to get to the required angle (and sometimes face backwards),
        or to rotate fully forwards to the correct angle.
        :param rotation_mode: False to rotate to nearest possible, True to
        rotate forwards to the required angle."""
        self.absolute_rotation = rotation_mode

    def set_velocity(self, vx, vy):
        """Set the x and y components of the desired module velocity, relative
        to the robot.
        :param vx: desired x velocity, m/s (x is forward on the robot)
        :param vy: desired y velocity, m/s (y is left on the robot)
        """
        self.vx = vx
        self.vy = vy

        # calculate straight line velocity and angle of motion
        velocity = math.hypot(self.vx, self.vy)
        direction = constrain_angle(math.atan2(self.vy, self.vx))

        # if we have a really low velocity, don't do anything. This is to
        # prevent stuff like joystick whipping back and changing the module
        # direction
        if velocity < 0.01:
            return

        if self.absolute_rotation:
            # Calculate a delta to from the module's current setpoint (wrapped
            # to between +-pi), representing required rotation to get to our
            # desired angle
            delta = constrain_angle(direction - self.module_sp_radians)
        else:
            # TODO: Test this code path on the actual robot (we have only tested absolute mode)

            # figure out the most efficient way to get the module to the desired direction
            current_heading = constrain_angle(self.module_sp_radians)
            delta = self.min_angular_displacement(current_heading, direction)

        # Please note, this is *NOT WRAPPED* to +-pi, because if wrapped the module
        # will unwind
        direction_to_set_radians = (self.module_sp_radians+delta)
        # convert the direction to encoder counts to set as the closed-loop setpoint
        direction_to_set = (direction_to_set_radians * self.STEER_COUNTS_PER_RADIAN
                + self.steer_enc_offset)
        self.steer_motor.set(direction_to_set)

        if not self.absolute_rotation:
            # logic to only move the modules when we are close to the corret angle
            # TODO: Test this code path on the actual robot (we have only tested absolute mode)
            direction_error = constrain_angle(self.module_sp_radians - direction)
            if abs(direction_error) < math.pi / 6.0:
                # if we are nearing the correct angle with the module forwards
                self.drive_motor.set(velocity*self.drive_velocity_to_native_units)
            elif abs(direction_error) > math.pi - math.pi / 6.0 and not self.absolute_rotation:
                # if we are nearing the correct angle with the module backwards
                self.drive_motor.set(-velocity*self.drive_velocity_to_native_units)
            else:
                self.drive_motor.set(0)
        else:
            self.drive_motor.set(velocity*self.drive_velocity_to_native_units)

    @property
    def module_sp_radians(self):
        """Read the current direction from the controller setpoint, and convert
        to radians"""
        setpoint = self.steer_motor.getSetpoint()
        return float(setpoint - self.steer_enc_offset) / self.STEER_COUNTS_PER_RADIAN

    @staticmethod
    def min_angular_displacement(current, target):
        """Return the minimum (signed) angular displacement to get from :param current:
        to :param target:. In radians."""
        target = constrain_angle(target)
        opp_target = constrain_angle(target + math.pi)
        current = constrain_angle(current)
        diff = constrain_angle(target - current)
        opp_diff = constrain_angle(opp_target - current)

        if abs(diff) < abs(opp_diff):
            return diff
        return opp_diff

def constrain_angle(angle):
    """Wrap :param angle: to between +pi and -pi"""
    return math.atan2(math.sin(angle), math.cos(angle))
