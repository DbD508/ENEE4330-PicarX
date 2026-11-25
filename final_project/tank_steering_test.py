from picarx import Picarx
import time

# Modified obstacle avoidance settings [Davey's Version (2026-11-08)]
POWER = 30
SAFE_DISTANCE = 50
DANGER_DISTANCE = 30

def torque_vectored_steering(severity: float, direction: str):
	"""Executes a torque-vectored steering maneuver. The intention of this function is to reduce the turning radius of the picar as much as possible.
		Args:
			severity: x in X={-100:100}, X =! 0. Specifies the severity of the steering. Positive values will result in net forward motion, while negative vaues will result in net backward motion. Note: if abs(severity)>100, input will be clamped. If severity = 0, function will not run.
			direction: 'left' produces a left turn, while 'right' produces a right turn (relative to forward motion).

		Returns:
			None. This is a hardware command

		Raises:
			NameError: 'direction' is not 'right' or 'left'
			TypeError: 'severity' is not 'float' or 'direction' is not 'string'
    """
    # Error Handling
    if severity == 0:
		return
    if type(severity) != float or !=int:
		raise TypeError('Invalid function input: allowable datatypes for severity are int or float')
	if type(direction) != str:
		raise TypeError('Invalid function input: allowable datatype for direction is string')
	if direction != 'right' and direction != 'left':
		raise NameError('Invalid function input: valid directions are left and right')

	# 				"IO table" 
	#		(F=forward, R=Reverse, l=left, r=right)
	#			F-l		F-r		R-l		R-r
	# Wheels	l		r		r		l
	# M_right	F		R		F		R
	# M_left	R		F		R		F
	
	# If this is broken out into individual logical equations,
		# Output: wheels left. Input: desired direction
			# Wheels,l = F-l or R-r							(1)
		# Output: M_right forward. Input: desired direction
			# M_right,F = l									(2)
		# Output: M_left forward. Input: desired direction
			# M_left,F = r									(3)

		# First, severity must be clamped to +/- 100
	severity_clamped=max(-100, min(severity, 100))
		# We will assign 1 = forward, -1 = reverse
	severity_sign = severity*(severity**2)**(-1/2) # severity/abs(severity)=sign(severity)
		# Now we assign -1 = left and 1 = right
	if direction == 'right':
		angle_modifier = 1
	else:
		angle_modifier = -1
		
		# Referring to (1), and with the knowledge that negative servo 
		# angle corresponds to "wheels, left", we can create a simple 
		# assignment for the steering angle
	angle = angle_modifier*severity_sign*DIR_MAX	# macro encodes +/-30 degree limit
	px.set_dir_servo_angle(angle)				# Set steering servo angle
	
	# The motors are mounted physically opposing one another, so setting
	# the power of each side with the same sign will result in rotation.
	# However, to compensate for the fact that this is not true tank
	# steering, the inner wheel's power will be increased relative to 
	# the outer wheel. Expanding the "IO Table" alreay developed:
	
	# 				"Augmented IO table" 
	#		(F=forward, R=Reverse, l=left, r=right)
	#			F-l		F-r		R-l		R-r
	# M_right	F		R+		F+		R
	# M_left	R+		F		R		F+

	# If we assign a variable P_ml to the condition "increased motor 
	# power, left" , we can draw out a relationship where:
	
	# P_ml = F-l + R-r							(4)
	# P_mr = F-r + R-l							(5)
	
	# Now, using the logical variables for steering which assigned 1 to
	# Forward and right, and -1 to Reverse and left:	
	left_power  = severity_clamped*((angle_modifier + severity_sign)*0.2+(angle_modifier - severity_sign)*0.8)
	right_power = severity_clamped*((direction + severity_sign)*0.2+(direction - severity_sign)*0.8)
	
	px.set_motor_speed(1, left_power)	# The motors are mounted physically opposing one another.
	px.set_motor_speed(2, right_power)	# Setting the speeds equal results in counter rotation
	
	# It has been decided that time.sleep() will be set to a fixed 0.5 second.
	# This allows for sensor checks while still giving the vehicle enough time to move.
		############################################################################
		# In the future it may be a good idea to integrate sensor readings into this
		# loop. Maybe even turn the loop into its own fis control routine
		############################################################################ 
	time.sleep(0.5)
	px.stop()
	
    def main():
    px = Picarx()
    
    try:
        while True:
                    px.set_dir_servo_angle(0)
                    px.forward(POWER/2)
                    time.sleep(1.5)
                    
                    torque_vectored_steering(80, 'right')
                    torque_vectored_steering(80, 'left')
                    torque_vectored_steering(-80, 'right')
                    torque_vectored_steering(-80, 'left')
                    
    except KeyboardInterrupt:
        print("Stopping obstacle avoidance")

    finally:
        # Stop motors and center steering
        px.stop()
        px.set_dir_servo_angle(0)
        print("Robot stopped safely")


if __name__ == "__main__":
    main()
