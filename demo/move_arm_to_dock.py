import numpy as np
import nestbox
import asyncio
from enum import Enum
import json
import time
import sys
import os
from typing import List, Tuple
from nestbox.metrology import PolynomialMapper
# there is a module called roarm in the directory by the same name, up one level from the absolute directory where the current file can be found
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'roarm')))
from roarm import RoArmAPI
from scipy.linalg import orthogonal_procrustes
from scipy.optimize import minimize


# import roarm modules found in ../roarm
import sys, os
roarm_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'roarm'))
sys.path.append(roarm_path)

class State(Enum):
    INITIALIZING = 0
    TAKING_DOCK_MEASUREMENTS = 1
    MOVING_ARM_TO_START = 2
    MEASURING_ARM_START = 2.5
    LOOP_MOVING_ARM = 3
    LOOP_MEASURING_ARM = 3.5
    FINAL_APPROACH = 4
    PLACING_OBJECT = 5
    COMPLETED = 6

class ArmDockAlignmentStateMachine:
    ROARM_HOME = [0.32, 0, 0.23] # x, y, z in meters

    def __init__(self):
        self.state = State.INITIALIZING
        self.dock_hand_points = []
        self.polynomial_approximator = None
        self.dock_measurements = []
        self.arm = None
        self.trajectory_index = 0
        self.arm_position = None
        self.destination_point = None
        self.max_move_step = 0.03
        self.consecutive_on_target = 0
        self.target_reached_threshold = 5  # Number of consecutive steps to consider target reached
        self._prev_meas = None

    def run(self):
        while self.state != State.COMPLETED:
            print(f"State: {self.state}")
            if self.state == State.INITIALIZING:
                self.initialize()
            elif self.state == State.TAKING_DOCK_MEASUREMENTS:
                self.take_dock_measurements()
            elif self.state == State.MOVING_ARM_TO_START:
                self.move_arm_to_start()
            elif self.state == State.MEASURING_ARM_START:
                self.measure_arm_start()
            elif self.state == State.LOOP_MOVING_ARM:
                self.move_arm_to_next()
            elif self.state == State.LOOP_MEASURING_ARM:
                self.measure_arm()
            elif self.state == State.FINAL_APPROACH:
                self.final_approach()
            elif self.state == State.PLACING_OBJECT:
                self.place_object()

    def initialize(self): # state INITIALIZING
        self._load_dock_hand_points()
        self._load_polynomial_approximator()
        self._create_coordinate_systems()
        self._submit_initial_measurements()
        testpoint = [0.32, 0, 0.23]
        proxypoint = self._map_arm_point_to_proxy(testpoint)
        print(f"Test point {testpoint} maps to {proxypoint}")
        invpoint = self._map_proxy_point_to_arm(proxypoint)
        print(f"Proxy point {proxypoint} maps back to {invpoint}")
        print(f"Error is {np.linalg.norm(testpoint - invpoint)}")
        self.state = State.TAKING_DOCK_MEASUREMENTS

    def _load_dock_hand_points(self):
        # Loads expected positions of hand joint points from file
        try:
            with open('data/dock_hand_positions.json', 'r') as f:
                data = json.load(f)
                # these are ordered from base to tip, thumb to pinky, 19 points total, in a right-handed coordinate system
                # this is the same order as the measurements from VR, except the ones from VR have two extra points,
                # one for a hand root position represented as a separate feature (root-pose/position) and one at the
                # start of the tracking point list representing the base of the thumb/bottom of the palm on the
                # thumb side, which is not in the dock_hand_points list, and also, the VR points are in a left-handed cs
                # this is a left hand
                # format is [
                # {"X": -198.454, "Y": -13.092, "Z": 34.045}
                # {"X": -210.303, "Y": -21.092, "Z": 55.271}
                # {"X": -222.944, "Y": -25.092, "Z": 72.030}...
                # ]
                self.dock_hand_points = np.array([[point['X'], point['Y'], point['Z']] for point in data])/1000 # Convert to meters
        except FileNotFoundError:
            raise FileNotFoundError("Dock hand positions file 'data/dock_hand_positions.json' not found.")

    def _load_polynomial_approximator(self):
        # Load calibration data
        try:
            with open('data/calibration_data.json', 'r') as f:
                data = json.load(f)
        except FileNotFoundError:
            raise FileNotFoundError("Calibration data file 'data/calibration_data.json' not found.")

        # Prepare data for the mapper
        arm_cal_setpoints = []
        vr_measurements = []
        for measurement in data['measurements'].values():
            arm_cal_setpoints.append(measurement['setpoint'])
            vr_measurements.append(measurement['vr_root_point'])

        # Convert to numpy arrays
        arm_cal_setpoints = np.array(arm_cal_setpoints)
        vr_measurements = np.array(vr_measurements)

        # Switch handedness
        vr_measurements[:, 0] *= -1

        # Center the data
        arm_cal_setpoints_mean = np.mean(arm_cal_setpoints, axis=0)
        vr_measurements_mean = np.mean(vr_measurements, axis=0)
        arm_cal_setpoints_centered = arm_cal_setpoints - arm_cal_setpoints_mean
        vr_measurements_centered = vr_measurements - vr_measurements_mean

        # Procrustes best fit
        A, _ = orthogonal_procrustes(vr_measurements_centered, arm_cal_setpoints_centered)
        if np.linalg.det(A) < 0:
            print('ORTHOGONAL PROCRUSTES: Determinant of A is negative. Something is very wrong.')
            exit()

        # Transform the vr measurements using the orthogonal matrix and translation
        best_fit_vr_measurements = np.dot(vr_measurements_centered, A)

        # Create and fit the polynomial mapper
        self.polynomial_approximator = PolynomialMapper(degree=4)
        self.polynomial_approximator.fit(arm_cal_setpoints_centered, best_fit_vr_measurements)

        # Store the mean for later use in transformations
        self.arm_cal_setpoints_mean = arm_cal_setpoints_mean

        print("Polynomial approximator loaded successfully.")

        if '--plot-poly-map' in sys.argv:
            def visualize_data(arm_cal_setpoints, vr_measurements, poly_mapper, lerp_mapper):
                import matplotlib.pyplot as plt

                fig = plt.figure(figsize=(12, 10))
                ax = fig.add_subplot(111, projection='3d')
                ax.grid(False)

                # Plot arm_cal_setpoints
                ax.scatter(arm_cal_setpoints[:, 0], arm_cal_setpoints[:, 1], arm_cal_setpoints[:, 2],
                        c='blue', label='Setpoints', alpha=0.6)

                # Plot VR measurements
                ax.scatter(vr_measurements[:, 0], vr_measurements[:, 1], vr_measurements[:, 2],
                        c='red', label='VR Measurements', alpha=0.6)

                # Generate test points
                test_points = []
                for _ in range(0):
                    n = 2
                    selected_indices = np.random.choice(len(arm_cal_setpoints), n)
                    selected_points = arm_cal_setpoints[selected_indices]
                    test_points.append(np.sum(selected_points, axis=0) / n)
                #also put in the 8 min/max corners of the data set
                x_min, x_max = arm_cal_setpoints[:, 0].min(), arm_cal_setpoints[:, 0].max()
                y_min, y_max = arm_cal_setpoints[:, 1].min(), arm_cal_setpoints[:, 1].max()
                z_min, z_max = arm_cal_setpoints[:, 2].min(), arm_cal_setpoints[:, 2].max()
                test_points.extend(np.array([[x_min, y_min, z_min], [x_min, y_min, z_max], [x_min, y_max, z_min], [x_min, y_max, z_max],
                                            [x_max, y_min, z_min], [x_max, y_min, z_max], [x_max, y_max, z_min], [x_max, y_max, z_max]]))
                test_points = np.array(test_points)

                # Apply polynomial mapping
                poly_results = poly_mapper.map(test_points)

                # Apply linear interpolation mapping
                lerp_results = lerp_mapper.map(test_points)

                # Plot test points and their mappings
                ax.scatter(test_points[:, 0], test_points[:, 1], test_points[:, 2],
                        c='green', s=100, label='Test Points')
                ax.scatter(poly_results[:, 0], poly_results[:, 1], poly_results[:, 2],
                        c='purple', s=100, label='Polynomial Map')
                ax.scatter(lerp_results[:, 0], lerp_results[:, 1], lerp_results[:, 2],
                        c='orange', s=100, label='Linear Interpolation')

                # Draw lines from test points to their mapped positions
                for i in range(len(test_points)):
                    ax.plot([test_points[i, 0], poly_results[i, 0]],
                            [test_points[i, 1], poly_results[i, 1]],
                            [test_points[i, 2], poly_results[i, 2]], 'purple', linestyle='--')
                    ax.plot([test_points[i, 0], lerp_results[i, 0]],
                            [test_points[i, 1], lerp_results[i, 1]],
                            [test_points[i, 2], lerp_results[i, 2]], 'orange', linestyle='--')

                # Ranges for the grids
                x_range = (x_min, x_max)
                y_range = (y_min, y_max)
                z_range = (z_min, z_max)

                # Generate and plot grids
                n_points = 7  # Number of points along each axis
                n_levels = 3   # Number of grid levels along each axis

                def generate_grids(x_range, y_range, z_range, n_points=10, n_levels=3):
                    grids = []
                    x_min, x_max = x_range
                    y_min, y_max = y_range
                    z_min, z_max = z_range

                    x_levels = np.linspace(x_min, x_max, n_levels)
                    y_levels = np.linspace(y_min, y_max, n_levels)
                    z_levels = np.linspace(z_min, z_max, n_levels)

                    # x-y grids at fixed z levels
                    for z_level in z_levels:
                        x_grid, y_grid = np.meshgrid(np.linspace(x_min, x_max, n_points),
                                                    np.linspace(y_min, y_max, n_points))
                        z_grid = np.full_like(x_grid, z_level)
                        grids.append((x_grid, y_grid, z_grid))

                    # x-z grids at fixed y levels
                    for y_level in y_levels:
                        x_grid, z_grid = np.meshgrid(np.linspace(x_min, x_max, n_points),
                                                    np.linspace(z_min, z_max, n_points))
                        y_grid = np.full_like(x_grid, y_level)
                        grids.append((x_grid, y_grid, z_grid))

                    # y-z grids at fixed x levels
                    for x_level in x_levels:
                        y_grid, z_grid = np.meshgrid(np.linspace(y_min, y_max, n_points),
                                                    np.linspace(z_min, z_max, n_points))
                        x_grid = np.full_like(y_grid, x_level)
                        grids.append((x_grid, y_grid, z_grid))

                    return grids

                grids = generate_grids(x_range, y_range, z_range, n_points=n_points, n_levels=n_levels)

                # Plot grids in input space (light blue)
                for x_grid, y_grid, z_grid in grids:
                    n_grid_points = x_grid.shape[0]
                    for i in range(n_grid_points):
                        ax.plot(x_grid[i, :], y_grid[i, :], z_grid[i, :],
                                color='lightblue', linewidth=0.5)
                    for j in range(n_grid_points):
                        ax.plot(x_grid[:, j], y_grid[:, j], z_grid[:, j],
                                color='lightblue', linewidth=0.5)

                # Map the grid points and plot in output space (light red)
                for x_grid, y_grid, z_grid in grids:
                    # Stack grid points into (N, 3) array
                    grid_points = np.vstack([x_grid.ravel(), y_grid.ravel(), z_grid.ravel()]).T
                    # Map grid points
                    mapped_points = poly_mapper.map(grid_points)
                    # Reshape back to grid shape
                    x_mapped = mapped_points[:, 0].reshape(x_grid.shape)
                    y_mapped = mapped_points[:, 1].reshape(y_grid.shape)
                    z_mapped = mapped_points[:, 2].reshape(z_grid.shape)
                    n_grid_points = x_mapped.shape[0]
                    for i in range(n_grid_points):
                        ax.plot(x_mapped[i, :], y_mapped[i, :], z_mapped[i, :],
                                color='lightcoral', linewidth=0.5)
                    for j in range(n_grid_points):
                        ax.plot(x_mapped[:, j], y_mapped[:, j], z_mapped[:, j],
                                color='lightcoral', linewidth=0.5)

                ax.set_xlabel('X')
                ax.set_ylabel('Y')
                ax.set_zlabel('Z')
                ax.set_title('Collected Data and Mapping Visualization')
                ax.legend()

                plt.tight_layout()
                plt.show()
            visualize_data(arm_cal_setpoints_centered, best_fit_vr_measurements, self.polynomial_approximator, self.polynomial_approximator)

    def _create_coordinate_systems(self):
        nestbox.create_coordinate_system('vr')
        nestbox.create_coordinate_system('armproxy')
        nestbox.create_coordinate_system('dock')

    def _submit_initial_measurements(self):
        for i, point in enumerate(self.dock_hand_points):
            nestbox.add_measurement(
                feature=f'nestbox:feature/hand/@dockhand/trackingpoint/{i}/position',
                cs='dock',
                mean=point.tolist(),
                covariance=[[4e-6, 0, 0], [0, 4e-6, 0], [0, 0, 4e-6]] # 2mm standard deviation, meaning +-4mm is 95% confidence interval
            )
        # add one measurement for arm home position
        nestbox.add_measurement(
            feature='nestbox:feature/hand/@roarmhandhome/root-pose/position',
            cs='armproxy',
            mean=self._map_arm_point_to_proxy(self.ROARM_HOME).tolist(),
            covariance=[[0.001, 0, 0], [0, 0.001, 0], [0, 0, 0.001]]
        )

    def _map_arm_point_to_proxy(self, arm_point: List[float]) -> List[float]:
        # Map an arm point to a VR point using the polynomial approximator, which maps centered arm points to the best-fit proxy space,
        # and which was constructed already including a handedness switch
        arm_point_centered = np.array(arm_point) - self.arm_cal_setpoints_mean
        proxy_point = self.polynomial_approximator.map(np.array([arm_point_centered]))[0]
        return proxy_point
    
    def _map_proxy_point_to_arm(self, proxy_point: List[float]) -> List[float]: # use minimize
        def objective(arm_point):
            point = self._map_arm_point_to_proxy(arm_point)
            return np.linalg.norm(point - proxy_point)
        result = minimize(objective, self.ROARM_HOME, method='Nelder-Mead')
        return result.x

    def _get_new_hand_measurement_in_righthand_vr(self):
        # Get the new hand measurement from VR or return None, None if there is not a new measurement since the last call
        # return list format is [root position, actual tracking point 0, actual tracking point 1, ...]
        try:
            # This is a bit of a hack. we're just using nestbox as a go-between to get the measurements from VR, then resubmitting them to nestbox
            # running nestbox daemon with --live flag will automatically populate measurements with these features ('lefthand' is the stream id)
            meas_response = nestbox.get_current_measurement('vr', 'nestbox:feature/hand/lefthand/root-pose/position')
            root_pos = np.array(meas_response['mean'])
            if np.linalg.norm(root_pos) < 0.0001: # Lost tracking
                time.sleep(1)
                return None, None
            root_pos[0] *= -1 # switch handedness
            if self._prev_meas is None: # always discard first measurement
                self._prev_meas = root_pos
                time.sleep(1)
                return None, None
            if self._prev_meas is not None and np.allclose(self._prev_meas, root_pos): # discard if no change
                time.sleep(1)
                return None, None
            self._prev_meas = root_pos
            print(f"Got measurement for root position: {root_pos}")
            tracking_poss = []
            for i in range(20):
                meas_response = nestbox.get_current_measurement('vr', f'nestbox:feature/hand/lefthand/trackingpoint/{i}/position')
                tracking_pos = np.array(meas_response['mean'])
                tracking_pos[0] *= -1 # switch handedness
                tracking_poss.append(tracking_pos)
            print(f"Got measurements for tracking points: {tracking_poss}")
        except Exception as e:
            print(f"Error getting measurement: {e}")
            time.sleep(1)
            return None, None
        return root_pos, tracking_poss

    def take_dock_measurements(self): # state TAKING_DOCK_MEASUREMENTS
        root_pos, tracking_poss = self._get_new_hand_measurement_in_righthand_vr()
        if tracking_poss is None:
            return
        # the first tracking point is the base of the thumb, which is not in the dock_hand_points list
        # we will use the dock hand ordering convention, meaning that index 0 is the "knuckle" of the thumb, which is shifted by 1 from the VR ordering
        tracking_poss.pop(0)
        # Submit measurements to nestbox
        for i, tracking_pos in enumerate(tracking_poss):
            nestbox.add_measurement(
                feature=f'nestbox:feature/hand/@dockhand/trackingpoint/{i}/position',
                cs='vr',
                mean=tracking_pos.tolist(),
                covariance=[[9e-6, 0, 0], [0, 9e-6, 0], [0, 0, 9e-6]] # 3mm standard deviation, meaning +-6mm is 95% confidence interval
            )
        self._prev_meas = None
        self.state = State.MOVING_ARM_TO_START

    def _initialize_arm_connection(self):
        if self.arm is not None:
            return
        # Initialize the arm
        roarm_host = '10.1.20.227' if '--roarm-host' not in sys.argv else sys.argv[sys.argv.index('--roarm-host')+1]
        self.arm = RoArmAPI(roarm_host, 8659)

    async def _move_arm_async(self, position: List[float]):
        self._initialize_arm_connection()
        await self.arm.connect()
        result = await self.arm.move_interp(*position, speed=0.1)
        print(f"Moved to {position}")
        print(f"Result from arm: {result}")
        self.arm_position = position
        await self.arm.close()

    def _move_arm(self, position: List[float]):
        asyncio.run(self._move_arm_async(position))

    def move_arm_to_start(self): # state MOVING_ARM_TO_START
        self._move_arm(self.ROARM_HOME)
        time.sleep(3)
        # After moving:
        self.state = State.MEASURING_ARM_START

    def measure_arm_start(self): # state MEASURING_ARM_START
        arm_home_measurement, _ = self._get_new_hand_measurement_in_righthand_vr()
        if arm_home_measurement is None:
            return
        nestbox.add_measurement(
            feature='nestbox:feature/hand/@roarmhandhome/root-pose/positionxxx', #TODO: fix this feature name
            cs='vr',
            mean=arm_home_measurement.tolist(),
            covariance=[[1e-4, 0, 0], [0, 1e-4, 0], [0, 0, 1e-4]] # 1cm standard deviation, meaning +-2cm is 95% confidence interval (TODO: tighten this up if we can)
        )
        nestbox.start_alignment()
        self.state = State.LOOP_MOVING_ARM

    def move_arm_to_next(self): # state LOOP_MOVING_ARM
        self.trajectory_index += 1
        new_destination_point = self._calculate_destination_point()
        destination_vector = new_destination_point - self.arm_position
        if np.linalg.norm(destination_vector) < 0.01: # if we're already close to the destination, record that we're on target
            self.consecutive_on_target += 1
        else:
            self.consecutive_on_target = 0
        if self.consecutive_on_target >= self.target_reached_threshold:
            self.state = State.FINAL_APPROACH
            return
        # new_trajectory_point = self.arm_position + (new_destination_point - self.arm_position) * 0.03 # a small percentage of the way to the destination
        # actually let's use a constant step size cap
        if np.linalg.norm(destination_vector) > self.max_move_step:
            new_trajectory_point = self.arm_position + destination_vector / np.linalg.norm(destination_vector) * self.max_move_step
        else:
            new_trajectory_point = new_destination_point
        # tell nestbox where we can expect the arm to be in VR, but in the basis of the arm
        # (or really, the basis of wherever the vr origin happened to be when calibration measurements were taken,
        # OR EVEN wherever that origin ended up after some procrustes/regularization, i.e. the proxy cs) (it's all rather abstract) (just trust Einstein)
        nestbox.add_measurement(
            feature=f'nestbox:feature/hand/@roarmhandtrajectory_{self.trajectory_index}/root-pose/position',
            cs='armproxy',
            mean=self._map_arm_point_to_proxy(new_trajectory_point).tolist(),
            covariance=[[4e-6, 0, 0], [0, 4e-6, 0], [0, 0, 4e-6]] # 2mm standard deviation, meaning +-4mm is 95% confidence interval TODO experimenting with this
        )
        self._move_arm(new_trajectory_point)
        time.sleep(3)
        self.state = State.LOOP_MEASURING_ARM

    def measure_arm(self): # state LOOP_MEASURING_ARM
        root_pos, tracking_poss = self._get_new_hand_measurement_in_righthand_vr()
        if root_pos is None:
            return
        nestbox.add_measurement(
            feature=f'nestbox:feature/hand/@roarmhandtrajectory_{self.trajectory_index}/root-pose/position',
            cs='vr',
            mean=root_pos.tolist(),
            #covariance=[[1e-4, 0, 0], [0, 1e-4, 0], [0, 0, 1e-4]] # 1cm standard deviation, meaning +-2cm is 95% confidence interval (TODO: tighten this up if we can)
            covariance=[[4e-6, 0, 0], [0, 4e-6, 0], [0, 0, 4e-6]] # 2mm standard deviation, meaning +-4mm is 95% confidence interval TODO experimenting with this
        )
        time.sleep(5)
        self.state = State.LOOP_MOVING_ARM

    def _calculate_destination_point(self) -> List[float]:
        dock_destination_point = np.array([0, 0.032, 0.129719])
        proxy_destination_point = nestbox.from_cs('dock').to_cs('armproxy').convert(dock_destination_point)
        return self._map_proxy_point_to_arm(proxy_destination_point)

    def final_approach(self): # state FINAL_APPROACH
        time.sleep(5)
        dock_approach_point = np.array([0, -0.014, 0.129719])
        arm_approach_point = nestbox.from_cs('dock').to_cs('armproxy').convert(dock_approach_point)
        self._move_arm(self._map_proxy_point_to_arm(arm_approach_point))
        self.state = State.PLACING_OBJECT

    def place_object(self): # state PLACING_OBJECT
        dock_place_point = np.array([0, -0.014, 0.090])
        arm_place_point = nestbox.from_cs('dock').to_cs('armproxy').convert(dock_place_point)
        self._move_arm(self._map_proxy_point_to_arm(arm_place_point))
        self.state = State.COMPLETED

if __name__ == "__main__":
    state_machine = ArmDockAlignmentStateMachine()
    state_machine.run()