import numpy as np
import control

# Define the LQR function using control.lqr
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
    
    # Normalize the rotation matrix to ensure it has determinant 1
    u, _, vh = np.linalg.svd(R)  # Singular Value Decomposition
    R_normalized = np.dot(u, vh)

    pitch_angle = np.arcsin(R_normalized[0,0]) + 1.5707
    
    # Determine the sign of the pitch angle based on R[2, 1]
    if R_normalized[0, 2]*R_normalized[1,2] < 0:
        pitch_angle = pitch_angle  # Tilt is backward
    else:
        pitch_angle = -pitch_angle

    return pitch_angle / 3

def sysCall_init():
    sim = require('sim')

    # Initialize scene objects (joints, wheels, etc.)
    left_motor = sim.getObject('/left_joint')
    right_motor = sim.getObject('/right_joint')
    bot_body = sim.getObject('/body')
    
    prismatic_joint_handle = sim.getObject('/body/Prismatic_joint')
    gripper_joint_handle = sim.getObject('/body/arm_joint')
    
    self.gripper_speed = 0
    self.arm_speed = 0

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

    # LQR cost matrices (for tuning system behavior)
    Q = np.diag([2.7, 6, 10, 4])  # State weighting matrix
    R = np.array([[1.25]])         # Control effort weighting matrix
    # Calculate the LQR controller gain K
    K = lqr(A, B, Q, R)
    # Store parameters in the global simulation dictionary
    self.K = K
    self.left_motor = left_motor
    self.right_motor = right_motor
    self.prismatic_joint_handle = prismatic_joint_handle
    self.gripper_joint_handle = gripper_joint_handle
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
    
    self.desired_gripper_speed = 0
    self.desired_arm_speed = 0
    
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
    
    speed_right = self.desired_wheel_speed_right
    speed_left = self.desired_wheel_speed_left
    
    desired_angle_right = self.desired_wheel_angle_right
    desired_angle_left = self.desired_wheel_angle_left

    orientation_quat = sim.getObjectQuaternion(bot_body, -1)  # Quaternion relative to world
    pitch_angle = calculate_angle_from_quaternion(orientation_quat)

    wheel_velocity_right = sim.getJointVelocity(right_motor)
    wheel_velocity_left = sim.getJointVelocity(left_motor)
    
    if turn == 0:
        wheel_velocity_right = wheel_velocity_left

    # Calculate time step
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
    X_desired_left = np.array([desired_angle_left, 0, 0, 0]).reshape((4, 1))

    # Calculate control input U = -K * (X - X_desired)
    error_right = X_right - X_desired_right
    error_left = X_left - X_desired_left
    
    U_right = -np.dot(K, error_right)
    U_left = -np.dot(K, error_left)

    # Apply control with saturation to prevent excessive torque
    max_torque = 2.5
    I_total = 0.0001065 
    mapping_coef = 2.5

    angular_velocity_right = U_right[0, 0] / mapping_coef + speed_right
    angular_velocity_left = U_left[0, 0] / mapping_coef + speed_left
    
    print(round(speed_right / 0.07))


    # Apply calculated angular velocity to the wheels
    sim.setJointTargetVelocity(left_motor, angular_velocity_left)
    sim.setJointTargetVelocity(right_motor, angular_velocity_right)
    
    # Apply gripper and arm speeds
    sim.setJointTargetVelocity(self.prismatic_joint_handle, self.desired_gripper_speed)
    sim.setJointTargetVelocity(self.gripper_joint_handle, self.desired_arm_speed)

def sysCall_sensing():
    sim = require('sim')
    
    # Define parameters for movement
    forward_speed_increment = 0.28  # Increment for moving forward
    turn_speed = 0.07
    turn_speed_2 = -0.07
    turn = self.turn 

    # Get the message, and check if a key is pressed
    message, data, data2 = sim.getSimulatorMessage()

    if message == sim.message_keypress:
        if data[0] == 2007:  # Up arrow key (move forward)
            self.desired_wheel_angle_left += forward_speed_increment
            self.desired_wheel_angle_right += forward_speed_increment
            
        elif data[0] == 2008:  # Down arrow key (move backward)
            self.desired_wheel_angle_left -= forward_speed_increment
            self.desired_wheel_angle_right -= forward_speed_increment
        elif data[0] == 2009:  # Left arrow key (turn left)
            self.desired_wheel_speed_left += turn_speed_2
            self.desired_wheel_speed_right += turn_speed
        elif data[0] == 2010:  # Right arrow key (turn right)
            self.desired_wheel_speed_left += turn_speed
            self.desired_wheel_speed_right += turn_speed_2
    
        elif data[0] == 113:  # 'Q' key for the gripper to close
            self.desired_gripper_speed += 0.02
        elif data[0] == 101:  # 'E' key for the gripper to open
            self.desired_gripper_speed -= 0.02
        elif data[0] == 119:  # 'W' key for the arm to go up
            self.desired_arm_speed -= 0.3
        elif data[0] == 115:  # 'S' key for the arm to go down
            self.desired_arm_speed += 0.3
        
        