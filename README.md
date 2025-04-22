# E-Yantra-Self-Balancing-Bot-
Self Balancing Bot Simulation on Coppeliasim

[YOUTUBE_LINK](https://youtu.be/oYKoDTpXtBs)

🛠️ Project Overview: Two-Wheeled Self-Balancing Robot (Simulation in CoppeliaSim)
This project presents a fully simulated two-wheeled self-balancing robot developed within the CoppeliaSim robotics simulation platform. The core focus of the project was to design a robot capable of maintaining its balance using advanced control strategies, while also enabling smooth, stable movement and manipulation tasks. The simulation environment was leveraged to test dynamic behaviors, optimize controllers, and validate all functionalities outlined in Stage 1 of a robotics competition. All control logic, dynamics modeling, and interactions were completed in the virtual simulation, making this a comprehensive proof of concept.

⚙️ System Dynamics and Control Design
The dynamics of the robot were modeled using Lagrangian mechanics, which allowed for a physics-based representation of its motion. This provided a set of state-space equations that described how the system responds to control inputs. These equations were central to developing an optimal control strategy using a Linear Quadratic Regulator (LQR). The LQR controller was designed to maintain balance by minimizing deviations from the desired posture while controlling wheel velocities efficiently. The controller continuously evaluated the robot’s state and applied real-time corrections to keep it upright.

🎯 Orientation Estimation Using Quaternions
To ensure accurate balancing and orientation feedback, the robot’s pitch angle—its tilt relative to the ground—was computed using quaternions. Quaternions are a compact and stable representation of 3D orientation that avoids the gimbal lock issues associated with Euler angles. In the simulation, the robot’s orientation was retrieved in quaternion form relative to the world frame. This quaternion was then mathematically converted into a rotation matrix, from which the pitch angle was extracted. This angle served as a vital input to the control system, allowing it to determine the robot’s lean direction and apply corrective motion to regain balance.

🤖 Robotic Arm and Pick-and-Place Mechanism
In addition to balancing and navigation, the robot was equipped with a manipulator arm featuring a prismatic joint and a gripper. This enabled the robot to interact with objects within its environment. The gripper could open and close, while the arm could raise or lower vertically, mimicking basic pick-and-place operations. The arm was controlled using direct velocity commands mapped to keyboard inputs. This allowed the user to command the robot to grasp and lift objects, demonstrating mobile manipulation capabilities while the robot continued to maintain balance—a non-trivial task that adds to the complexity and realism of the simulation.

🧠 Real-Time Control and Motion Commands
The robot supported real-time motion control based on user input. Through keyboard commands, the robot could move forward, backward, and turn left or right, all while staying upright. Each keypress was translated into a desired change in wheel angles or speeds, which was then processed by the controller to compute the required motor commands. The system also accounted for the robot’s pitch and wheel angular velocity, integrating these in the feedback loop to ensure smooth and stable transitions between different movement modes. The ability to move and rotate while preserving balance was crucial in demonstrating dynamic control effectiveness.

✅ Stage 1 Completion and Performance
This simulation project successfully completed all tasks outlined in Stage 1 of the robotics competition. The robot could stabilize from varying initial conditions, navigate through user-defined trajectories, perform responsive turning, and interact with its environment using its robotic arm. Each task was accomplished while the robot maintained a balanced posture, reflecting a successful integration of motion planning, control theory, and real-time actuation in a virtual yet realistic setting.
