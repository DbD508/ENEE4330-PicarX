from picarx import Picarx
from time import sleep, monotonic

# Modified from SunFounder's original line-tracking example.
# Designed to slow down for sharp corners and reacquire the next line.
px = Picarx()

STRAIGHT_SPEED = 10
TURN_SPEED = 6
TURN_ANGLE = 22
RECOVERY_SPEED = 5
RECOVERY_ANGLE = 28
RECOVERY_TIMEOUT = 0.8
LOOP_DELAY = 0.02

last_state = 'stop'


def get_status(val_list):
    # Keep the original SunFounder interpretation of grayscale readings.
    state = px.get_line_status(val_list)
    if state == [0, 0, 0]:
        return 'stop'
    elif state[1] == 1:
        return 'forward'
    elif state[0] == 1:
        return 'right'
    elif state[2] == 1:
        return 'left'
    return 'stop'


def search_for_line(direction):
    """Briefly continue the last turn to find the next tape segment."""
    px.stop()
    if direction not in ('left', 'right'):
        return False

    angle = RECOVERY_ANGLE if direction == 'left' else -RECOVERY_ANGLE
    px.set_dir_servo_angle(angle)
    deadline = monotonic() + RECOVERY_TIMEOUT

    while monotonic() < deadline:
        status = get_status(px.get_grayscale_data())
        if status != 'stop':
            px.stop()
            print('Line recovered:', status)
            return True
        px.forward(RECOVERY_SPEED)
        sleep(LOOP_DELAY)

    px.stop()
    print('Line not found; stopping')
    return False


def main():
    global last_state
    try:
        while True:
            readings = px.get_grayscale_data()
            state = get_status(readings)
            print('Grayscale:', readings, 'State:', state)

            if state == 'forward':
                px.set_dir_servo_angle(0)
                px.forward(STRAIGHT_SPEED)
            elif state == 'left':
                last_state = 'left'
                px.set_dir_servo_angle(TURN_ANGLE)
                px.forward(TURN_SPEED)
            elif state == 'right':
                last_state = 'right'
                px.set_dir_servo_angle(-TURN_ANGLE)
                px.forward(TURN_SPEED)
            else:
                if not search_for_line(last_state):
                    break
            sleep(LOOP_DELAY)
    except KeyboardInterrupt:
        print('Interrupted')
    finally:
        px.stop()
        px.set_dir_servo_angle(0)
        print('Robot stopped')


if __name__ == '__main__':
    main()
