from picarx import Picarx
import time

# Modified obstacle avoidance settings [Davey's Version (2026-11-08)]
POWER = 30
SAFE_DISTANCE = 50
DANGER_DISTANCE = 30

def main():
    px = Picarx()
    
    try:
        while True:
                    px.set_dir_servo_angle(0)
                    px.forward(POWER/2)
                    time.sleep(1.5)
                    px.set_dir_servo_angle(30)
                    px.set_motor_speed(1, 80)
                    px.set_motor_speed(2, 80)
                    time.sleep(1)
                    px.set_dir_servo_angle(-30)
                    px.set_motor_speed(1, -80)
                    px.set_motor_speed(2, -80)
                    time.sleep(1)
                    
    except KeyboardInterrupt:
        print("Stopping obstacle avoidance")

    finally:
        # Stop motors and center steering
        px.stop()
        px.set_dir_servo_angle(0)
        print("Robot stopped safely")


if __name__ == "__main__":
    main()
