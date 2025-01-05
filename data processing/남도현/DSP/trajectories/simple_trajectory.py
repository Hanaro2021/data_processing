from .imu_trajectory import IMUTrajectory
import numpy as np
from scipy.spatial.transform import Rotation


class SimpleTrajectory(IMUTrajectory):
    def __init__(self, acc, dt, gravity, gyro, rotator0):
        super().__init__(acc=acc, dt=dt, rotator=None, gravity=gravity)
        self.gyro = gyro
        self.rotator0 = rotator0
        self.rotator = self.generate_rotator()

    def generate_rotator(self):
        """Generate rotation matrices using gyro data."""
        rotator = [self.rotator0]
        for i in range(len(self.gyro['x'])):
            # Get angular velocities
            wx = self.gyro['x'][i]
            wy = self.gyro['y'][i]
            wz = self.gyro['z'][i]
            
            # Create rotation matrix for this step
            rot = Rotation.from_rotvec([wx * self.dt, wy * self.dt, wz * self.dt])
            
            # Combine with previous rotation
            prev_rot = Rotation.from_quat(rotator[-1])
            new_rot = rot * prev_rot
            
            rotator.append(new_rot.as_quat())
            
        return rotator

    def calculate_trajectory(self, launch_index, touchdown_index):
        super().calculate_trajectory(launch_index, touchdown_index)

    def plot_trajectory(self, ax, color, label):
        super().plot_trajectory(ax, color, label)
