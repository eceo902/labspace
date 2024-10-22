import numpy as np
import json
import os
import time
from enum import Enum
import nestbox

class State(Enum):
    INITIALIZING = 1
    MOVING_ARM = 2
    WAITING_FOR_SETTLE = 3
    CAPTURING_MEASUREMENT = 4
    SAVING_DATA = 5
    PLANNING_NEXT_POINT = 6

import numpy as np
import json
import os
import time
from enum import Enum
import nestbox
from roarm import RoArmAPI
import asyncio

class State(Enum):
    INITIALIZING = 1
    MOVING_ARM = 2
    WAITING_FOR_SETTLE = 3
    CAPTURING_MEASUREMENT = 4
    SAVING_DATA = 5
    PLANNING_NEXT_POINT = 6

class DataCollectionCoordinator:
    def __init__(self, file_name, grid_spacing=0.05, host='10.1.20.227'):
        self.file_name = file_name
        self.grid_spacing = grid_spacing
        self.state = State.INITIALIZING
        self.current_point_id = None
        self.data = self.load_data()
        self.grid_points, self.position_queue = self.generate_position_queue()
        self._prev_meas = None
        self.host = host
        self.arm = None

    def in_bounds(self, point):
        assert len(point) == 3, "point must be a 3-tuple"
        return 0.2 < np.linalg.norm(point) < 0.5 and point[1] > -0.2 and point[2] > 0

    def generate_grid_points(self):
        x = np.arange(-0.5, 0.5, self.grid_spacing)
        y = np.arange(-0.5, 0.5, self.grid_spacing)
        z = np.arange(0, 0.5, self.grid_spacing)
        
        nx, ny, nz = len(x), len(y), len(z)
        
        grid = np.array(np.meshgrid(x, y, z)).T.reshape(-1, 3)
        indices = np.array(np.meshgrid(range(nx), range(ny), range(nz))).T.reshape(-1, 3)
        
        dtype = [('coords', float, (3,)), ('indices', int, (3,))]
        grid_with_indices = np.empty(grid.shape[0], dtype=dtype)
        grid_with_indices['coords'] = grid
        grid_with_indices['indices'] = indices
        
        valid_points = grid_with_indices[np.array([self.in_bounds(point) for point in grid_with_indices['coords']])]
        
        return valid_points

    def load_data(self):
        if os.path.exists(self.file_name):
            with open(self.file_name, 'r') as f:
                data = json.load(f)
            if 'grid_spacing' in data:
                self.grid_spacing = data['grid_spacing']
            return data
        else:
            return {'grid_spacing': self.grid_spacing, 'measurements': {}}

    def save_data(self):
        with open(self.file_name, 'w') as f:
            json.dump(self.data, f, indent=2)

    def generate_position_queue(self):
        grid_points = self.generate_grid_points()
        measured_indices = set(self.data['measurements'].keys())
        queue = []
        for point in grid_points:
            coords, (i, j, k) = point['coords'], point['indices']
            point_id = f"{i}_{j}_{k}"
            if point_id not in measured_indices:
                queue.append((point_id, coords))
        print(f"Generated {len(queue)} points for measurement")
        # sort the queue by y value to avoid going back and forth across the xz plane, which makes the robot arm go around the long way very fast
        queue.sort(key=lambda x: x[1][1])
        return grid_points, queue

    def get_vr_measurements(self):
        while True:
            try:
                meas_response = nestbox.get_current_measurement('vr', 'nestbox:feature/hand/lefthand/root-pose/position')
                root_pos = np.array(meas_response['mean'])
                if self._prev_meas is not None and np.allclose(self._prev_meas, root_pos):
                    time.sleep(1)
                    continue
                self._prev_meas = root_pos
                # also go get tracking point measurements of the form nestbox:feature/hand/lefthand/trackingpoint/0/position or /1/position etc from 0 to 19
                tracking_poss = [root_pos]
                for i in range(20):
                    meas_response = nestbox.get_current_measurement('vr', f'nestbox:feature/hand/lefthand/trackingpoint/{i}/position')
                    tracking_pos = np.array(meas_response['mean'])
                    tracking_poss.append(tracking_pos)
                break
            except Exception as e:
                print(f"Error getting measurement: {e}")
                time.sleep(1)
        for i, pos in enumerate(tracking_poss):
            print(f"Got measurement {i}: {pos}")
        return tracking_poss

    async def initialize_arm(self):
        self.arm = RoArmAPI(self.host, 8659)
        await self.arm.connect()

    async def close_arm(self):
        if self.arm:
            await self.arm.close()

    async def command_robot_arm(self, position):
        if not self.arm:
            await self.initialize_arm()
        
        x, y, z = position
        try:
            result = await self.arm.move_interp(x, y, z)
            print(f"Robot arm moved to position: {position}")
            print(f"Result: {result}")
        except Exception as e:
            print(f"Error moving robot arm: {e}")

    async def run(self):
        try:
            await self.initialize_arm()
            while self.position_queue:
                self.state = State.PLANNING_NEXT_POINT
                self.current_point_id, position = self.position_queue.pop(0)

                self.state = State.MOVING_ARM
                await self.command_robot_arm(position)

                self.state = State.WAITING_FOR_SETTLE
                await asyncio.sleep(2)  # Adjust as needed

                self.state = State.CAPTURING_MEASUREMENT
                vr_measurements = self.get_vr_measurements()

                self.state = State.SAVING_DATA
                self.data['measurements'][self.current_point_id] = {
                    'setpoint': position.tolist(),
                    'vr_root_point': vr_measurements[0].tolist(),
                    'vr_tracking_points': [pos.tolist() for pos in vr_measurements[1:]]
                }
                self.save_data()

                print(f"Measured point {self.current_point_id}: setpoint {position}, VR measurement {vr_measurements}")

            print("Data collection complete")
        finally:
            await self.close_arm()

if __name__ == "__main__":
    coordinator = DataCollectionCoordinator('measurement_data.json', grid_spacing=0.1)
    asyncio.run(coordinator.run())







# class DataCollectionCoordinator:
#     def __init__(self, file_name, grid_spacing=0.05):
#         self.file_name = file_name
#         self.grid_spacing = grid_spacing
#         self.state = State.INITIALIZING
#         self.current_point_id = None
#         self.data = self.load_data()
#         self.grid_points, self.position_queue = self.generate_position_queue()
#         self._prev_meas = None

#     def in_bounds(self, point):
#         assert len(point) == 3, "point must be a 3-tuple"
#         return 0.2 < np.linalg.norm(point) < 0.5 and point[1] > -0.2 and point[2] > 0

#     def generate_grid_points(self):
#         x = np.arange(-0.5, 0.5, self.grid_spacing)
#         y = np.arange(-0.5, 0.5, self.grid_spacing)
#         z = np.arange(0, 0.5, self.grid_spacing)
        
#         nx, ny, nz = len(x), len(y), len(z)
        
#         grid = np.array(np.meshgrid(x, y, z)).T.reshape(-1, 3)
#         indices = np.array(np.meshgrid(range(nx), range(ny), range(nz))).T.reshape(-1, 3)
        
#         dtype = [('coords', float, (3,)), ('indices', int, (3,))]
#         grid_with_indices = np.empty(grid.shape[0], dtype=dtype)
#         grid_with_indices['coords'] = grid
#         grid_with_indices['indices'] = indices
        
#         valid_points = grid_with_indices[np.array([self.in_bounds(point) for point in grid_with_indices['coords']])]
        
#         return valid_points

#     def load_data(self):
#         if os.path.exists(self.file_name):
#             with open(self.file_name, 'r') as f:
#                 data = json.load(f)
#             if 'grid_spacing' in data:
#                 self.grid_spacing = data['grid_spacing']
#             return data
#         else:
#             return {'grid_spacing': self.grid_spacing, 'measurements': {}}

#     def save_data(self):
#         with open(self.file_name, 'w') as f:
#             json.dump(self.data, f, indent=2)

#     def generate_position_queue(self):
#         grid_points = self.generate_grid_points()
#         measured_indices = set(self.data['measurements'].keys())
#         queue = []
#         for point in grid_points:
#             coords, (i, j, k) = point['coords'], point['indices']
#             point_id = f"{i}_{j}_{k}"
#             if point_id not in measured_indices:
#                 queue.append((point_id, coords))
#         print(f"Generated {len(queue)} points for measurement")
#         return grid_points, queue

#     def get_vr_measurements(self):
#         mean = None
#         while True:
#             try:
#                 meas_response = nestbox.get_current_measurement('vr', 'nestbox:feature/hand/lefthand/root-pose/position')
#                 mean = np.array(meas_response['mean'])
#                 if self._prev_meas is not None and np.allclose(self._prev_meas, mean):
#                     time.sleep(1)
#                     continue
#                 self._prev_meas = mean
#                 break
#             except Exception as e:
#                 print(f"Error getting measurement: {e}")
#                 time.sleep(1)
#         print(f"Got measurement: {mean}")
#         return mean

#     def command_robot_arm(self, position):
#         # Send command to robot arm
#         # This is a placeholder. Replace with actual robot arm control code.
#         print(f"Moving robot arm to position: {position}")

#     def run(self):
#         while self.position_queue:
#             self.state = State.PLANNING_NEXT_POINT
#             self.current_point_id, position = self.position_queue.pop(0)

#             self.state = State.MOVING_ARM
#             self.command_robot_arm(position)

#             self.state = State.WAITING_FOR_SETTLE
#             time.sleep(2)  # Adjust as needed

#             self.state = State.CAPTURING_MEASUREMENT
#             vr_measurement = self.get_vr_measurements()

#             self.state = State.SAVING_DATA
#             self.data['measurements'][self.current_point_id] = {
#                 'setpoint': position.tolist(),
#                 'vr_measurement': vr_measurement.tolist()
#             }
#             self.save_data()

#             print(f"Measured point {self.current_point_id}: setpoint {position}, VR measurement {vr_measurement}")

#         print("Data collection complete")


# if __name__ == "__main__":
#     coordinator = DataCollectionCoordinator('measurement_data.json', grid_spacing=0.05)
#     coordinator.run()