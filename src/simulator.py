import pybullet as p
import pybullet_data
import time
import math

class RobotSimulator:
    def __init__(self, urdf_path):
        self.physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        
        planeId = p.loadURDF("plane.urdf")
        self.robotId = p.loadURDF(urdf_path, [0, 0, 0], useFixedBase=True)
        
        self.joint_indices = {}
        for i in range(p.getNumJoints(self.robotId)):
            info = p.getJointInfo(self.robotId, i)
            joint_name = info[1].decode('utf-8')
            self.joint_indices[joint_name] = i
        
        print(f"✅ Simulator initialized with joints: {list(self.joint_indices.keys())}")

        # --- Store initial object position for reset ---
        self.initial_object_pos = [0.25, 0, 0.02]
        self.object_id = p.loadURDF("cube_small.urdf", self.initial_object_pos)
        self.gripped_constraint_id = None

    def set_joint_angle(self, joint_name, angle_degrees):
        if joint_name in self.joint_indices:
            angle_radians = math.radians(angle_degrees)
            p.setJointMotorControl2(
                self.robotId,
                self.joint_indices[joint_name],
                p.POSITION_CONTROL,
                targetPosition=angle_radians
            )

    def step(self):
        p.stepSimulation()
        time.sleep(1./240.)

    def close(self):
        p.disconnect()

    def control_gripper(self, grip_command):
        """Handles the logic for gripping and releasing objects."""
        # Use the gripper_base_link as the reference for creating the constraint.
        # We get its index from the wrist_joint, as it's the parent of the gripper base.
        gripper_link_index = self.joint_indices['wrist_joint']

        # --- LOGIC TO GRIP ---
        if grip_command == "CLOSE" and self.gripped_constraint_id is None:
            # Find objects near the gripper
            closest_points = p.getClosestPoints(self.robotId, self.object_id, distance=0.05, linkIndexA=gripper_link_index)
            
            if len(closest_points) > 0:
                print("✅ Object detected! Gripping.")
                # Get the position of the object relative to the gripper link
                link_state = p.getLinkState(self.robotId, gripper_link_index)
                link_pos, link_orn = link_state[0], link_state[1]
                
                obj_pos, obj_orn = p.getBasePositionAndOrientation(self.object_id)

                # Calculate the inverse transform of the gripper link
                inv_link_pos, inv_link_orn = p.invertTransform(link_pos, link_orn)
                
                # Calculate the object's position in the gripper's local frame
                obj_pos_in_link, obj_orn_in_link = p.multiplyTransforms(inv_link_pos, inv_link_orn, obj_pos, obj_orn)

                # Create a fixed constraint
                self.gripped_constraint_id = p.createConstraint(
                    parentBodyUniqueId=self.robotId,
                    parentLinkIndex=gripper_link_index,
                    childBodyUniqueId=self.object_id,
                    childLinkIndex=-1, # -1 for the base of the object
                    jointType=p.JOINT_FIXED,
                    jointAxis=[0, 0, 0],
                    parentFramePosition=obj_pos_in_link,
                    childFramePosition=[0, 0, 0],
                    parentFrameOrientation=obj_orn_in_link
                )

        # --- LOGIC TO RELEASE ---
        elif grip_command == "OPEN" and self.gripped_constraint_id is not None:
            print(" releasing grip.")
            p.removeConstraint(self.gripped_constraint_id)
            self.gripped_constraint_id = None

    def reset(self):
        """Resets the robot and object to their initial states."""
        print("🔄 Resetting simulation...")
        
        # 1. Remove any grip constraint that might exist
        if self.gripped_constraint_id is not None:
            p.removeConstraint(self.gripped_constraint_id)
            self.gripped_constraint_id = None
            
        # 2. Reset the cube to its starting position
        p.resetBasePositionAndOrientation(self.object_id, self.initial_object_pos, [0, 0, 0, 1])
        p.resetBaseVelocity(self.object_id, [0,0,0], [0,0,0]) # Stop any movement

        # 3. Reset all robot joints to a default "home" pose
        home_pose = {
            "base_joint": 0, "shoulder_joint": -45, "elbow_joint": 45,
            "wrist_joint": 45, "left_gripper_joint": 0, "right_gripper_joint": 0
        }
        for name, angle in home_pose.items():
            p.resetJointState(self.robotId, self.joint_indices[name], targetValue=math.radians(angle))
