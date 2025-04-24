import numpy as np
import control

#Define the LQR function using control.lqr
def lqr(A, B, Q, R):
    """
    LQR function to calculate gain matrix K using the control library.
    """
    # Ensure matrices are NumPy arrays
    A = np.array(A)
    B = np.array(B)
    Q = np.array(Q)
    R = np.array(R)

    # Calculate the LQR controller gain K using the control library
    K, S, E = control.lqr(A, B, Q, R)
    
    return K

def calculate_angle_from_quaternion(quaternion):
    """
    Calculates the pitch angle relative to the bot's initial x-axis (wheel axis),
    distinguishing the direction of tilt (left/right or forward/backward).
    """
    # Extract quaternion components
    q0, q1, q2, q3 = quaternion
    
    # Convert quaternion to rotation matrix
    R = np.array([
        [1 - 2 * (q2**2 + q3**2), 2 * (q1 * q2 - q3 * q0), 2 * (q1 * q3 + q2 * q0)],
        [2 * (q1 * q2 + q3 * q0), 1 - 2 * (q1**2 + q3**2), 2 * (q2 * q3 - q1 * q0)],
        [2 * (q1 * q3 - q2 * q0), 2 * (q2 * q3 + q1 * q0), 1 - 2 * (q1**2 + q2**2)]
    ])
    
    #print(R)
    
    # Normalize the rotation matrix to ensure it has determinant 1
    #u, _, vh = np.linalg.svd(R)  # Singular Value Decomposition
    #R_normalized = np.dot(u, vh)

    #print(R_normalized)

    # Calculate the pitch angle as before
    #pitch_angle = np.arctan2(R_normalized[1,1], R_normalized[0,1]) + 1.57079632679
    #yaw_angle = np.arctan2(R_normalized[2,1], R_normalized[1,1]) + 3.14
    
    pitch_angle = np.arcsin(R[0,0]) + 1.5707
    
    
    # Determine the sign of the pitch angle based on R[2, 1]
    if R[0, 2]*R[1,2] < 0:
        pitch_angle = pitch_angle  # Tilt is backward
    else:
        pitch_angle = -pitch_angle
     
    #print(pitch_angle)

    return pitch_angle/2.717



def sysCall_init():
    sim = require('sim')

    # Initialize scene objects (joints, wheels, etc.)
    left_motor = sim.getObject('/left_joint')
    right_motor = sim.getObject('/right_joint')
    bot_body = sim.getObject('/body')

    # Define system matrices for the state-space representation
    A = np.array([
        [0, 0, 1, 0],
        [0, 0, 0, 1],
        [0, -91.0627, 0, 0],
        [0, 119.882, 0, 0]
    ])

    B = np.array([
       [0],
       [0],
     [11838.249],
     [-4686.797]
    ])
    

    # considering discretization sample time to be 0.01 s
    

    # LQR cost matrices (for tuning system behavior)
    Q = np.diag([2.5,6,10,4])  # State weighting matrix
    R = np.array([[1.1]])     # Control effort weighting matrix
    # Calculate the LQR controller gain K
    K = lqr(A, B, Q, R)
    # Store parameters in the global simulation dictionary
    self.K = K
    self.left_motor = left_motor
    self.right_motor = right_motor
    self.bot_body = bot_body
    self.prev_wheel_angle_right = 0
    self.prev_wheel_angle_left = 0
    self.prev_pitch_angle = 0
    self.prev_time = sim.getSimulationTime()
    
    self.desired_wheel_angle_left = 0
    self.desired_wheel_angle_right = 0
    
    self.wheel_angle = 0 
    self.turn = 0 
    
    self.desired_wheel_speed_right = 0
    self.desired_wheel_speed_left = 0
    
    
    self.desired_wheel_angle_left = 0
    self.desired_wheel_angle_right = 0
    # Initialize the initial joint velocity to zero for both motors
    sim.setJointTargetVelocity(left_motor, 0.0)
    sim.setJointTargetVelocity(right_motor, 0.0)

def sysCall_actuation():
    sim = require('sim')

    # Retrieve variables stored in self
    K = self.K
    left_motor = self.left_motor
    right_motor = self.right_motor
    bot_body = self.bot_body
    
    turn = self.turn 
    
    wheel_angle_right = self.prev_wheel_angle_right 
    wheel_angle_left = self.prev_wheel_angle_left
    
    
    speed_right =  self.desired_wheel_speed_right 
    speed_left = self.desired_wheel_speed_left
    
    #print(f"Left Wheel angle: {wheel_angle_left}")
    #print(f"Right Wheel angle  : {wheel_angle_right}")
    
    desired_angle_right = self.desired_wheel_angle_right
    desired_angle_left = self.desired_wheel_angle_left
    
    #print(f"Desired Left Wheel angle: {desired_angle_left}")
    #print(f"Desired Right Wheel angle  : {desired_angle_right}")
    
    previous_angle_right = self.prev_wheel_angle_right
    previous_angle_left = self.prev_wheel_angle_left
    
        
    orientation_quat = sim.getObjectQuaternion(bot_body, -1)  # Quaternion relative to world
    pitch_angle = calculate_angle_from_quaternion(orientation_quat)
    
    #orientation = sim.getObjectOrientation(bot_body,-1)
    
    #pitch_angle = -orientation[0]
    #pitch_angle_3 = -orientation[1]
    
    
    #print(pitch_angle)
    
    wheel_velocity_right = sim.getJointVelocity(right_motor)
    wheel_velocity_left = sim.getJointVelocity(left_motor)
    
    #delta = round(sim.getJointVelocity(left_motor), 2) - round(sim.getJointVelocity(right_motor), 2) 
   
    if turn == 0 :  
        wheel_velocity_right = wheel_velocity_left
    
    
    #print(f"Wheel angle is: {wheel_angle}")
    #print(f"Wheel velocity is: {wheel_velocity}")

    # Manually calculate cart angular velocity (change in pitch angle over time)
    current_time = sim.getSimulationTime()
    dt = current_time - self.prev_time
    cart_angular_velocity = (pitch_angle - self.prev_pitch_angle) / dt if dt > 0 else 0
    
    wheel_angle_right += wheel_velocity_right * dt
    wheel_angle_left += wheel_velocity_left * dt
    
    # Update stored values
    self.prev_wheel_angle_right = wheel_angle_right
    self.prev_wheel_angle_left = wheel_angle_left
    
    self.prev_pitch_angle = pitch_angle
    self.prev_time = current_time

    # State vector X and desired state X_desired
    X_right = np.array([wheel_angle_right, pitch_angle, wheel_velocity_right, cart_angular_velocity]).reshape((4, 1))
    X_left = np.array([wheel_angle_left, pitch_angle, wheel_velocity_left, cart_angular_velocity]).reshape((4, 1))
    
    X_desired_right = np.array([desired_angle_right, 0, 0, 0]).reshape((4, 1))
    X_desired_left = np.array([desired_angle_left, 0, 0 , 0]).reshape((4, 1))
    
    # print(X_desired_right)

    # Calculate control input U = -K * (X - X_desired)
    error_right = X_right - X_desired_right
    error_left = X_left - X_desired_left
    
    U_right = -np.dot(K, error_right)
    U_left = -np.dot(K, error_left)


    # Apply control with saturation to prevent excessive torque
    max_torque = 2.5
    # U = np.clip(U, -max_torque, max_torque)
    
    I_total = 0.0001065 
    
    mapping_coef = 2.5
    

    
    #print(f"Wheel angle is right: {wheel_angle_right}")
    #print(f"Wheel angle is left: {wheel_angle_left}")
    #print(f"Pitch angle is: {pitch_angle}")
    #print(f"Wheel velocity is: {wheel_velocity}")
    #print(f"Cart angular velocity is: {cart_angular_velocity}")
    # print(f"Cart angular velocity is: {cart_angular_velocity}")

    
    # Calculate angular velocity from torque
    # angular_velocity = U[0, 0] / I_total  # Where I_total is the effective moment of inertia
    angular_velocity_right = U_right[0, 0] / mapping_coef
    angular_velocity_left = U_left[0, 0] / mapping_coef
    
    #print(f"right: {wheel_velocity_right}")
    #print(f"left: {wheel_velocity_left}")
    
    angular_velocity_right = angular_velocity_right + speed_right 
    angular_velocity_left = angular_velocity_left + speed_left
     
    #print(angular_velocity_right)
    #print(angular_velocity_left) 
    
    print(round(speed_right / 0.07))
    
    # Apply calculated angular velocity to the wheels
    sim.setJointTargetVelocity(left_motor, angular_velocity_left)
    sim.setJointTargetVelocity(right_motor, angular_velocity_right)

    
def sysCall_sensing():
    sim = require('sim')

    # Define parameters for movement
    forward_speed_increment = 0.35  # Increment for moving forward
    turn_angle_increment = 0.5  # Small angle increment for turning
    turn_angle_increment_2 = 0.5
    #max_wheel_angle = 0.5        # Maximum angle setpoint limit
    
    #turn_speed = 0.01
    
    turn_speed = 0.07
    turn_speed_2 = -0.07
    turn = self.turn 
    #turn = 0
    
    # Get the message, and check if a key is pressed
    message, data, data2 = sim.getSimulatorMessage()

    if message == sim.message_keypress:
        if data[0] == 2007:  # Up arrow key (move forward)
            # Increment both wheels equally for forward movement
            self.desired_wheel_angle_left += forward_speed_increment
            self.desired_wheel_angle_right += forward_speed_increment
            
        elif data[0] == 2008:  # Down arrow key (move backward)
            # Decrement both wheels equally for backward movement
            self.desired_wheel_angle_left -= forward_speed_increment
            self.desired_wheel_angle_right -= forward_speed_increment
        elif data[0] == 2009:  # Left arrow key (turn left)
            # Increment right wheel and decrement left wheel for turning left
            #self.desired_wheel_angle_left -= turn_angle_increment
            #self.desired_wheel_angle_right += turn_angle_increment
            
            self.desired_wheel_speed_left += turn_speed
            self.desired_wheel_speed_right += turn_speed_2

            
        elif data[0] == 2010:  # Right arrow key (turn right)
            # Increment left wheel and decrement right wheel for turning right
            #self.desired_wheel_angle_left += turn_angle_increment
            #self.desired_wheel_angle_right -= turn_angle_increment
            
            self.desired_wheel_speed_left += turn_speed_2
            self.desired_wheel_speed_right += turn_speed


            
    if message == sim.message_keypress:
        
        if data[0] == 2009 or data[0] == 2010 :
            turn = 1 
            
    else :
        turn = 0 
         
    self.turn = turn    
    #print(turn)
    #print(self.desired_wheel_angle_right)
    #print(self.desired_wheel_angle_left)
    # Clamp the setpoints to avoid excessively large angles
    #self.desired_wheel_angle_left = np.clip(self.desired_wheel_angle_left, -max_wheel_angle, max_wheel_angle)
    #self.desired_wheel_angle_right = np.clip(self.desired_wheel_angle_right, -max_wheel_angle, max_wheel_angle)

    # Use desired wheel angle setpoints in the control loop (e.g., in sysCall_actuation)
    # The controller will attempt to reach these target angles smoothly


def sysCall_cleanup():
    pass