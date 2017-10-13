
from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController

GAS_DENSITY = 2.858 
ONE_MPH = 0.44704 # 1 mph ~ 0.44704 m/s

#Pamaters to be tuned
PID_KP = 0.2
PID_KI = 0.007
PID_KD = 3.0

LPF_TAU = 0.1
LPF_TS = 0.1

YAW_MIN_SPEED = 1.0

class Controller(object):
    def __init__(self, **kwargs):
        self.params = kwargs    

        #Note that throttle values passed to publish should be in the range 0 to 1. 
        #Brake values passed to publish should be in units of torque (N*m). 
        #The correct values for brake can be computed using the desired acceleration, weight of the vehicle, and wheel radius.
        self.torque = (self.params['vehicle_mass'] + self.params['fuel_capacity']*GAS_DENSITY) * self.params['wheel_radius']

        self.pid = PID(PID_KP, PID_KI, PID_KD, self.params['decel_limit'], self.params['accel_limit'])
        self.lpf= LowPassFilter(LPF_TAU, LPF_TS)
        self.yaw_controller = YawController(
            self.params['wheel_base'], self.params['steer_ratio'], YAW_MIN_SPEED, self.params['max_lat_accel'], self.params['max_steer_angle'])

    def control(self, dbw_enabled, twist_cmd_msg, current_velocity_msg, sample_time):
        if not dbw_enabled:
            self.pid.reset()
            return 0.0, 0.0, 0.0            

        proposed_linear = twist_cmd_msg.twist.linear.x
        proposed_angular = twist_cmd_msg.twist.angular.z

        cur_linear = current_velocity_msg.twist.linear.x
        #cur_angular = current_velocity_msg.twist.angular.z

        linear_error = (proposed_linear - cur_linear) * ONE_MPH

        throttle = self.pid.step(linear_error, sample_time)
        throttle = self.lpf.filt(throttle)

        brake = 0.0
        if throttle < 0.0:
            if -throttle > self.params['brake_deadband']:
                brake = -self.torque * throttle 
            throttle = 0.0
        
        steer = self.yaw_controller.get_steering(proposed_linear, proposed_angular, cur_linear)
        return throttle, brake, steer
