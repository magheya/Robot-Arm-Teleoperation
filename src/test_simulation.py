import pybullet as p
import pybullet_data
import time

# 1. Connect to the Physics Engine
# p.GUI means we want to see the simulation. Use p.DIRECT for no GUI.
physicsClient = p.connect(p.GUI)
print("✅ Simulation connected.")

# 2. Setup the Environment
# Add the pybullet_data path to find default assets like the ground plane
p.setAdditionalSearchPath(pybullet_data.getDataPath()) 
p.setGravity(0, 0, -9.81)

# Load a ground plane
planeId = p.loadURDF("plane.urdf")

# 3. Load Your Robot
# The starting position and orientation of the robot
startPos = [0, 0, 0]
startOrientation = p.getQuaternionFromEuler([0, 0, 0])

# The path to your URDF file.
# This assumes this script is in 'src' and the URDF is in 'src/sim_files'
robot_urdf_path = "src/sim_files/robot_arm.urdf"

try:
    # Load the URDF file and get the unique ID for the robot
    robotId = p.loadURDF(robot_urdf_path, startPos, startOrientation, useFixedBase=1)
    
    # Get the number of joints and print their names
    num_joints = p.getNumJoints(robotId)
    print(f"✅ Successfully loaded '{robot_urdf_path}' with {num_joints} joints.")
    for i in range(num_joints):
        joint_info = p.getJointInfo(robotId, i)
        print(f"  - Joint {i}: {joint_info[1].decode('utf-8')}")

except p.error as e:
    print(f"❌ Error loading URDF: {e}")
    print("Please check the path and the XML syntax of your URDF file.")
    p.disconnect()
    exit()


# 4. Run the Simulation
# This is an infinite loop to keep the simulation window open.
# Close the window to stop the script.
print("\n--- Simulation running. Close the window to exit. ---")
while p.isConnected():
    p.stepSimulation()
    time.sleep(1./240.) # Standard sleep time for real-time simulation

p.disconnect()
print("✅ Simulation disconnected.")