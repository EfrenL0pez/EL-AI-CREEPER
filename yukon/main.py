"""
El AI Creeper - Yukon
"""

from pimoroni_yukon import Yukon, SLOT1, SLOT4
from pimoroni_yukon.modules import BigMotorModule, QuadServoRegModule
import sys
import time

MOTOR_SLOT = SLOT1
SERVO_SLOT = SLOT4
STEERING_SERVO = 3
HEAD_SERVO = 0
STEERING_INVERT = -1
HEAD_INVERT = 1

def initialize():
    yukon = Yukon()
    motor_module = BigMotorModule()
    servo_module = QuadServoRegModule()
    yukon.register_with_slot(motor_module, MOTOR_SLOT)
    yukon.register_with_slot(servo_module, SERVO_SLOT)
    yukon.verify_and_initialise(allow_unregistered=True)
    yukon.enable_main_output()
    time.sleep(1)
    motor_module.enable()
    servo_module.enable()
    servo_module.servos[STEERING_SERVO].enable()
    servo_module.servos[HEAD_SERVO].enable()
    servo_module.servos[STEERING_SERVO].value(0)
    servo_module.servos[HEAD_SERVO].value(0)
    print("Ready")
    return yukon, motor_module, servo_module

def main():
    yukon, motor_module, servo_module = initialize()

    throttle = 0.0
    steering = 0.0
    
    # How long to hold before decay (in loops, ~20ms each)
    throttle_hold = 0
    steering_hold = 0
    HOLD_TIME = 15  # ~300ms before starting to decay

    try:
        while True:
            line = sys.stdin.readline().strip()
            
            if line:
                parts = line.split(',')
                if len(parts) == 3:
                    try:
                        cmd_throttle = float(parts[0])
                        cmd_steering = float(parts[1])
                        
                        # Force stop
                        if cmd_throttle == -99.0:
                            throttle = 0.0
                            steering = 0.0
                            throttle_hold = 0
                            steering_hold = 0
                        else:
                            # Throttle
                            if cmd_throttle > 0.5:  # W pressed
                                throttle = min(1.0, throttle + 0.05)
                                throttle_hold = HOLD_TIME
                            elif cmd_throttle < -0.5:  # S pressed
                                throttle = max(-1.0, throttle - 0.05)
                                throttle_hold = HOLD_TIME
                            else:  # No key
                                if throttle_hold > 0:
                                    throttle_hold -= 1  # Wait before decay
                                else:
                                    # Decay toward 0
                                    if throttle > 0:
                                        throttle = max(0, throttle - 0.03)
                                    elif throttle < 0:
                                        throttle = min(0, throttle + 0.03)
                            
                            # Steering
                            if cmd_steering > 0.5:  # A pressed
                                steering = min(90, steering + 3)
                                steering_hold = HOLD_TIME
                            elif cmd_steering < -0.5:  # D pressed
                                steering = max(-90, steering - 3)
                                steering_hold = HOLD_TIME
                            else:  # No key
                                if steering_hold > 0:
                                    steering_hold -= 1
                                else:
                                    if steering > 0:
                                        steering = max(0, steering - 2)
                                    elif steering < 0:
                                        steering = min(0, steering + 2)
                    except:
                        pass
            
            # Apply
            motor_module.motor.speed(throttle)
            servo_module.servos[STEERING_SERVO].value(steering * STEERING_INVERT)
            servo_module.servos[HEAD_SERVO].value(steering * HEAD_INVERT)

    except KeyboardInterrupt:
        pass
    finally:
        motor_module.disable()
        servo_module.disable()
        yukon.reset()

if __name__ == "__main__":
    main()
