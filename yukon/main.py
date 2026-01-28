"""
El AI Creeper - Yukon Motor Controller
======================================

This code receives commands from the Raspberry Pi via USB serial
and controls the motors and servos.

Command format from Pi: throttle,steering,head
Example: 0.500,-0.300,0.200

Requirements:
- Pimoroni Yukon with latest firmware
- Big Motor Module (for drive)
- Quad Servo Regulated Module (for steering + head)

Setup:
1. Flash Yukon firmware: https://github.com/pimoroni/yukon/releases/latest
2. Edit the CONFIGURATION section below to match YOUR slot setup
3. Copy this file to Yukon using Thonny
4. Connect Yukon to Pi via USB
"""

from pimoroni_yukon import Yukon, SLOT1, SLOT2, SLOT3, SLOT4, SLOT5, SLOT6
from pimoroni_yukon.modules import BigMotorModule, QuadServoRegModule
import sys
import time


# =============================================================================
# CONFIGURATION - CHANGE THESE TO MATCH YOUR SETUP
# =============================================================================

# Which slot is your Big Motor Module plugged into?
# Options: SLOT1, SLOT2, SLOT3, SLOT4, SLOT5, SLOT6
MOTOR_SLOT = SLOT1

# Which slot is your Quad Servo Module plugged into?
# Options: SLOT1, SLOT2, SLOT3, SLOT4, SLOT5, SLOT6
SERVO_SLOT = SLOT4

# Which servo port is your STEERING servo connected to?
# Options: 0, 1, 2, or 3
STEERING_SERVO = 3

# Which servo port is your HEAD servo connected to?
# Options: 0, 1, 2, or 3
HEAD_SERVO = 0

# Invert steering? Set to -1 if steering is backwards, 1 if normal
STEERING_INVERT = -1

# Invert head? Set to -1 if head is backwards, 1 if normal
HEAD_INVERT = 1

# =============================================================================
# END OF CONFIGURATION - Don't edit below unless you know what you're doing
# =============================================================================


def initialize():
    """Set up Yukon, motor, and servos"""
    
    print("Initializing Yukon...")
    yukon = Yukon()
    
    # Create module objects
    motor_module = BigMotorModule()
    servo_module = QuadServoRegModule()

    # Register modules with their slots
    yukon.register_with_slot(motor_module, MOTOR_SLOT)
    yukon.register_with_slot(servo_module, SERVO_SLOT)

    # Initialize and power on
    yukon.verify_and_initialise()
    yukon.enable_main_output()
    time.sleep(1)

    # Enable modules
    motor_module.enable()
    servo_module.enable()

    # Enable and center servos
    servo_module.servos[STEERING_SERVO].enable()
    servo_module.servos[HEAD_SERVO].enable()
    servo_module.servos[STEERING_SERVO].value(0)
    servo_module.servos[HEAD_SERVO].value(0)

    print("Yukon ready - waiting for commands")
    return yukon, motor_module, servo_module


def main():
    """Main loop - read serial commands and control motors/servos"""
    
    yukon, motor_module, servo_module = initialize()

    try:
        while True:
            # Read command from Pi via USB serial
            command = sys.stdin.readline().strip()
            
            if command:
                parts = command.split(',')
                
                # Expect 3 values: throttle, steering, head
                if len(parts) != 3:
                    continue

                try:
                    throttle = float(parts[0])
                    steering = float(parts[1])
                    head = float(parts[2])
                except ValueError:
                    continue

                # Apply to motor and servos
                motor_module.motor.speed(throttle)
                servo_module.servos[STEERING_SERVO].value(steering * STEERING_INVERT)
                servo_module.servos[HEAD_SERVO].value(head * HEAD_INVERT)

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("Stopping...")

    finally:
        # Clean shutdown
        motor_module.disable()
        servo_module.disable()
        yukon.reset()
        print("Yukon shutdown complete")


if __name__ == "__main__":
    main()
