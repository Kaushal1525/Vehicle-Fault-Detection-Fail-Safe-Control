import math
import random
import numpy as np
import matplotlib.pyplot as plt

from matplotlib.animation import FuncAnimation
from matplotlib.patches import Rectangle, Circle


# ===============================================================
# CONFIGURATION
# ===============================================================

DT = 0.05

SIMULATION_TIME = 60.0

ROAD_LENGTH = 220.0
ROAD_WIDTH = 18.0

LANE_WIDTH = 6.0

LANE_CENTERS = [
    3.0,
    9.0,
    15.0
]

VEHICLE_LENGTH = 4.8
VEHICLE_WIDTH = 2.2

INITIAL_SPEED = 14.0

NORMAL_ACCELERATION = 1.0

NORMAL_DECELERATION = 3.5

EMERGENCY_DECELERATION = 8.5

MIN_SAFE_SPEED = 0.0

GPS_NOISE = 1.8

IMU_NOISE = 0.15

CAMERA_RANGE = 45.0

LIDAR_RANGE = 60.0

WHEEL_NOISE = 0.12

FAULT_DETECTION_TIMEOUT = 1.5

SAFE_STOP_DISTANCE = 5.0


# ===============================================================
# FAULT WINDOWS
# ===============================================================
#
# The simulation intentionally injects faults.
#
# Each fault has:
#
#       START TIME
#       END TIME
#
# You can modify these values to test different scenarios.
# ===============================================================

FAULT_WINDOWS = {

    "GPS": (10.0, 17.0),

    "IMU": (19.0, 25.0),

    "CAMERA": (28.0, 35.0),

    "LIDAR": (38.0, 45.0),

    "WHEEL": (47.0, 53.0),

    "BRAKE": (55.0, 60.0)
}


# ===============================================================
# UTILITY FUNCTIONS
# ===============================================================

def clamp(
    value,
    minimum,
    maximum
):

    return max(
        minimum,
        min(value, maximum)
    )


def normalize_angle(angle):

    while angle > math.pi:

        angle -= 2.0 * math.pi

    while angle < -math.pi:

        angle += 2.0 * math.pi

    return angle


def is_fault_active(
    sensor,
    current_time
):

    if sensor not in FAULT_WINDOWS:

        return False

    start, end = FAULT_WINDOWS[
        sensor
    ]

    return (
        start
        <=
        current_time
        <=
        end
    )


# ===============================================================
# TRUE VEHICLE
# ===============================================================

class TrueVehicle:

    def __init__(self):

        self.x = 10.0

        self.y = LANE_CENTERS[1]

        self.velocity = INITIAL_SPEED

        self.heading = 0.0

        self.acceleration = 0.0

        self.steering = 0.0

        self.time = 0.0

        self.history_x = []

        self.history_y = []

        self.history_speed = []

    def update(
        self,
        acceleration
    ):

        self.acceleration = acceleration

        self.velocity += (
            acceleration
            * DT
        )

        self.velocity = clamp(
            self.velocity,
            0.0,
            18.0
        )

        self.x += (
            self.velocity
            *
            math.cos(
                self.heading
            )
            *
            DT
        )

        self.y += (
            self.velocity
            *
            math.sin(
                self.heading
            )
            *
            DT
        )

        self.x = clamp(
            self.x,
            5.0,
            ROAD_LENGTH - 5.0
        )

        self.y = clamp(
            self.y,
            1.0,
            ROAD_WIDTH - 1.0
        )

        self.history_x.append(
            self.x
        )

        self.history_y.append(
            self.y
        )

        self.history_speed.append(
            self.velocity
        )

        self.time += DT


# ===============================================================
# SIMULATED SENSOR
# ===============================================================

class Sensor:

    def __init__(
        self,
        name,
        reliability
    ):

        self.name = name

        self.reliability = reliability

        self.available = True

        self.last_update = 0.0

        self.failure_detected = False

        self.measurement_count = 0

        self.failure_count = 0

        self.confidence = reliability

    def update_health(
        self,
        current_time
    ):

        fault = is_fault_active(
            self.name,
            current_time
        )

        if fault:

            self.available = False

            self.failure_count += 1

            self.confidence = 0.0

        else:

            self.available = True

            self.confidence = (
                self.reliability
            )

        return self.available


# ===============================================================
# GPS
# ===============================================================

class GPSSensor(Sensor):

    def __init__(self):

        super().__init__(
            "GPS",
            0.90
        )

    def read(
        self,
        vehicle,
        current_time
    ):

        self.update_health(
            current_time
        )

        if not self.available:

            return None

        self.measurement_count += 1

        x = (
            vehicle.x
            +
            random.gauss(
                0.0,
                GPS_NOISE
            )
        )

        y = (
            vehicle.y
            +
            random.gauss(
                0.0,
                GPS_NOISE
            )
        )

        return np.array(
            [
                x,
                y
            ]
        )


# ===============================================================
# IMU
# ===============================================================

class IMUSensor(Sensor):

    def __init__(self):

        super().__init__(
            "IMU",
            0.95
        )

        self.bias = 0.0

    def read(
        self,
        vehicle,
        current_time
    ):

        self.update_health(
            current_time
        )

        if not self.available:

            return None

        self.measurement_count += 1

        self.bias += (
            random.gauss(
                0,
                0.002
            )
        )

        acceleration = (
            vehicle.acceleration
            +
            self.bias
            +
            random.gauss(
                0,
                IMU_NOISE
            )
        )

        gyro = (
            vehicle.heading
            +
            random.gauss(
                0,
                math.radians(1.0)
            )
        )

        return {
            "acceleration": acceleration,
            "heading": gyro
        }


# ===============================================================
# CAMERA
# ===============================================================

class CameraSensor(Sensor):

    def __init__(self):

        super().__init__(
            "CAMERA",
            0.92
        )

    def read(
        self,
        vehicle,
        obstacles,
        current_time
    ):

        self.update_health(
            current_time
        )

        if not self.available:

            return None

        self.measurement_count += 1

        detections = []

        for obstacle in obstacles:

            dx = (
                obstacle["x"]
                -
                vehicle.x
            )

            dy = (
                obstacle["y"]
                -
                vehicle.y
            )

            d = math.sqrt(
                dx * dx
                +
                dy * dy
            )

            if d <= CAMERA_RANGE:

                detections.append(
                    {
                        "x": obstacle["x"],
                        "y": obstacle["y"],
                        "distance": d
                    }
                )

        return detections


# ===============================================================
# LIDAR
# ===============================================================

class LiDARSensor(Sensor):

    def __init__(self):

        super().__init__(
            "LIDAR",
            0.98
        )

    def read(
        self,
        vehicle,
        obstacles,
        current_time
    ):

        self.update_health(
            current_time
        )

        if not self.available:

            return None

        self.measurement_count += 1

        detections = []

        for obstacle in obstacles:

            dx = (
                obstacle["x"]
                -
                vehicle.x
            )

            dy = (
                obstacle["y"]
                -
                vehicle.y
            )

            d = math.sqrt(
                dx * dx
                +
                dy * dy
            )

            if d <= LIDAR_RANGE:

                detections.append(
                    {
                        "x": obstacle["x"],
                        "y": obstacle["y"],
                        "distance": d
                    }
                )

        return detections


# ===============================================================
# WHEEL ENCODER
# ===============================================================

class WheelEncoderSensor(Sensor):

    def __init__(self):

        super().__init__(
            "WHEEL",
            0.94
        )

        self.distance = 0.0

    def read(
        self,
        vehicle,
        current_time
    ):

        self.update_health(
            current_time
        )

        if not self.available:

            return None

        self.measurement_count += 1

        distance = (
            vehicle.velocity
            *
            DT
        )

        distance += random.gauss(
            0.0,
            WHEEL_NOISE
        )

        self.distance += distance

        return {
            "distance": self.distance,
            "velocity": (
                vehicle.velocity
                +
                random.gauss(
                    0.0,
                    0.25
                )
            )
        }


# ===============================================================
# BRAKE SYSTEM
# ===============================================================

class BrakeSystem(Sensor):

    def __init__(self):

        super().__init__(
            "BRAKE",
            0.98
        )

    def apply(
        self,
        requested_deceleration,
        current_time
    ):

        self.update_health(
            current_time
        )

        if not self.available:

            return 0.0

        return -abs(
            requested_deceleration
        )


# ===============================================================
# SENSOR FUSION / LOCALIZATION
# ===============================================================

class LocalizationSystem:

    def __init__(self):

        self.x = 10.0

        self.y = LANE_CENTERS[1]

        self.velocity = INITIAL_SPEED

        self.heading = 0.0

        self.source = "INITIAL"

        self.confidence = 0.5

        self.history_x = []

        self.history_y = []

    def update(
        self,
        gps,
        imu,
        wheel,
        current_time
    ):

        sources = []

        estimates_x = []

        estimates_y = []

        # -------------------------------------------------------
        # GPS
        # -------------------------------------------------------

        if gps is not None:

            estimates_x.append(
                gps[0]
            )

            estimates_y.append(
                gps[1]
            )

            sources.append(
                "GPS"
            )

        # -------------------------------------------------------
        # Wheel odometry
        # -------------------------------------------------------

        if wheel is not None:

            velocity = (
                wheel["velocity"]
            )

            self.velocity = (
                0.8 * self.velocity
                +
                0.2 * velocity
            )

            sources.append(
                "WHEEL"
            )

        # -------------------------------------------------------
        # IMU
        # -------------------------------------------------------

        if imu is not None:

            self.velocity += (
                imu["acceleration"]
                *
                DT
            )

            self.velocity = clamp(
                self.velocity,
                0,
                20
            )

            self.heading = normalize_angle(
                0.9 * self.heading
                +
                0.1 * imu["heading"]
            )

            sources.append(
                "IMU"
            )

        # -------------------------------------------------------
        # Position estimation
        # -------------------------------------------------------

        if estimates_x:

            # Sensor fusion weighted mean

            self.x = np.mean(
                estimates_x
            )

            self.y = np.mean(
                estimates_y
            )

        else:

            # Dead reckoning

            self.x += (
                self.velocity
                *
                math.cos(
                    self.heading
                )
                *
                DT
            )

            self.y += (
                self.velocity
                *
                math.sin(
                    self.heading
                )
                *
                DT
            )

        # -------------------------------------------------------
        # Localization source
        # -------------------------------------------------------

        if (
            gps is not None
            and
            imu is not None
            and
            wheel is not None
        ):

            self.source = (
                "GPS + IMU + WHEEL"
            )

            self.confidence = 0.98

        elif (
            gps is not None
            and
            imu is not None
        ):

            self.source = (
                "GPS + IMU"
            )

            self.confidence = 0.90

        elif (
            imu is not None
            and
            wheel is not None
        ):

            self.source = (
                "IMU + WHEEL"
            )

            self.confidence = 0.75

        elif gps is not None:

            self.source = "GPS ONLY"

            self.confidence = 0.65

        elif imu is not None:

            self.source = "IMU DEAD RECKONING"

            self.confidence = 0.55

        elif wheel is not None:

            self.source = "WHEEL ODOMETRY"

            self.confidence = 0.45

        else:

            self.source = "NO LOCALIZATION"

            self.confidence = 0.10

        self.x = clamp(
            self.x,
            0,
            ROAD_LENGTH
        )

        self.y = clamp(
            self.y,
            0,
            ROAD_WIDTH
        )

        self.history_x.append(
            self.x
        )

        self.history_y.append(
            self.y
        )


# ===============================================================
# FAULT DETECTION SYSTEM
# ===============================================================

class FaultDetectionSystem:

    def __init__(self):

        self.active_faults = []

        self.fault_history = []

        self.last_fault = "NONE"

        self.system_confidence = 1.0

    def evaluate(
        self,
        sensors,
        current_time
    ):

        self.active_faults = []

        for sensor in sensors:

            if not sensor.available:

                self.active_faults.append(
                    sensor.name
                )

        self.system_confidence = 1.0

        for sensor in sensors:

            self.system_confidence *= (
                sensor.confidence
            )

        if self.active_faults:

            self.last_fault = (
                self.active_faults[-1]
            )

            self.fault_history.append(
                (
                    current_time,
                    list(
                        self.active_faults
                    )
                )
            )

        else:

            self.last_fault = "NONE"

        return self.active_faults


# ===============================================================
# SAFETY MANAGER
# ===============================================================

class SafetyManager:

    def __init__(self):

        self.mode = (
            "FULL AUTONOMY"
        )

        self.previous_mode = (
            "FULL AUTONOMY"
        )

        self.emergency_braking = False

        self.safe_stop = False

        self.critical_fault = False

        self.steering_available = True

        self.brake_available = True

        self.mode_history = []

    def evaluate(
        self,
        faults,
        vehicle_speed
    ):

        self.previous_mode = (
            self.mode
        )

        self.brake_available = (
            "BRAKE" not in faults
        )

        self.critical_fault = (
            "BRAKE" in faults
            and
            vehicle_speed > 2.0
        )

        self.steering_available = (
            "IMU" not in faults
        )

        # -------------------------------------------------------
        # Multiple critical sensors
        # -------------------------------------------------------

        critical_sensor_count = 0

        for sensor in [
            "GPS",
            "IMU",
            "CAMERA",
            "LIDAR",
            "WHEEL"
        ]:

            if sensor in faults:

                critical_sensor_count += 1

        # -------------------------------------------------------
        # Brake failure
        # -------------------------------------------------------

        if "BRAKE" in faults:

            self.mode = (
                "CRITICAL FAULT"
            )

            self.emergency_braking = True

        # -------------------------------------------------------
        # Multiple navigation failures
        # -------------------------------------------------------

        elif critical_sensor_count >= 4:

            self.mode = (
                "SAFE STOP"
            )

            self.emergency_braking = True

        # -------------------------------------------------------
        # IMU + wheel failure
        # -------------------------------------------------------

        elif (
            "IMU" in faults
            and
            "WHEEL" in faults
        ):

            self.mode = (
                "SAFE STOP"
            )

            self.emergency_braking = True

        # -------------------------------------------------------
        # Single / limited failure
        # -------------------------------------------------------

        elif faults:

            self.mode = (
                "DEGRADED AUTONOMY"
            )

            self.emergency_braking = False

        else:

            self.mode = (
                "FULL AUTONOMY"
            )

            self.emergency_braking = False

        self.mode_history.append(
            self.mode
        )

        return self.mode


# ===============================================================
# AUTONOMOUS CONTROLLER
# ===============================================================

class AutonomousController:

    def __init__(self):

        self.acceleration_command = (
            NORMAL_ACCELERATION
        )

        self.steering_command = 0.0

    def calculate(
        self,
        vehicle,
        obstacles,
        camera_data,
        lidar_data,
        safety,
        current_time
    ):

        # -------------------------------------------------------
        # Emergency safety behavior
        # -------------------------------------------------------

        if safety.mode == "CRITICAL FAULT":

            return -1.5

        if safety.mode == "SAFE STOP":

            return -EMERGENCY_DECELERATION

        # -------------------------------------------------------
        # Detect obstacle
        # -------------------------------------------------------

        closest_distance = float(
            "inf"
        )

        for data in [
            camera_data,
            lidar_data
        ]:

            if data is None:

                continue

            for detection in data:

                dx = (
                    detection["x"]
                    -
                    vehicle.x
                )

                dy = (
                    detection["y"]
                    -
                    vehicle.y
                )

                # Only objects ahead

                if dx <= 0:

                    continue

                if abs(
                    dy
                ) > 3.0:

                    continue

                d = math.sqrt(
                    dx * dx
                    +
                    dy * dy
                )

                closest_distance = min(
                    closest_distance,
                    d
                )

        # -------------------------------------------------------
        # Emergency braking decision
        # -------------------------------------------------------

        if closest_distance < 8:

            if safety.brake_available:

                return (
                    -EMERGENCY_DECELERATION
                )

            else:

                return 0.0

        if closest_distance < 15:

            if safety.brake_available:

                return (
                    -5.0
                )

        if closest_distance < 25:

            if safety.brake_available:

                return (
                    -2.0
                )

        # -------------------------------------------------------
        # Degraded autonomy
        # -------------------------------------------------------

        if safety.mode == (
            "DEGRADED AUTONOMY"
        ):

            return 0.3

        # -------------------------------------------------------
        # Normal driving
        # -------------------------------------------------------

        if vehicle.velocity < INITIAL_SPEED:

            return 1.0

        return 0.0


# ===============================================================
# MAIN SIMULATION
# ===============================================================

class AutonomousSimulation:

    def __init__(self):

        self.time = 0.0

        self.vehicle = (
            TrueVehicle()
        )

        self.gps = GPSSensor()

        self.imu = IMUSensor()

        self.camera = CameraSensor()

        self.lidar = LiDARSensor()

        self.wheel = (
            WheelEncoderSensor()
        )

        self.brake = BrakeSystem()

        self.localization = (
            LocalizationSystem()
        )

        self.fault_detector = (
            FaultDetectionSystem()
        )

        self.safety = (
            SafetyManager()
        )

        self.controller = (
            AutonomousController()
        )

        # -------------------------------------------------------
        # Static / moving obstacles
        # -------------------------------------------------------

        self.obstacles = [

            {
                "x": 75.0,
                "y": 9.0,
                "speed": 5.0
            },

            {
                "x": 130.0,
                "y": 3.0,
                "speed": 6.0
            },

            {
                "x": 175.0,
                "y": 15.0,
                "speed": 4.0
            }
        ]

        # -------------------------------------------------------
        # History
        # -------------------------------------------------------

        self.time_history = []

        self.true_x = []

        self.true_y = []

        self.estimated_x = []

        self.estimated_y = []

        self.speed_history = []

        self.confidence_history = []

        self.mode_history = []

        self.localization_history = []

        self.fault_history = []

        self.active_fault_history = []

        self.gps_available_history = []

        self.imu_available_history = []

        self.camera_available_history = []

        self.lidar_available_history = []

        self.wheel_available_history = []

        self.brake_available_history = []

        self.position_error = []

    # -----------------------------------------------------------
    # Update obstacles
    # -----------------------------------------------------------

    def update_obstacles(self):

        for obstacle in self.obstacles:

            obstacle["x"] += (
                obstacle["speed"]
                *
                DT
            )

            if (
                obstacle["x"]
                >
                ROAD_LENGTH + 10
            ):

                obstacle["x"] = (
                    self.vehicle.x
                    +
                    random.uniform(
                        60,
                        100
                    )
                )

    # -----------------------------------------------------------
    # Apply brake
    # -----------------------------------------------------------

    def apply_control(
        self,
        command
    ):

        current_time = self.time

        if command < 0:

            if self.brake.available:

                brake_acceleration = (
                    self.brake.apply(
                        abs(command),
                        current_time
                    )
                )

                return (
                    brake_acceleration
                )

            else:

                # Brake failed
                return 0.0

        return command

    # -----------------------------------------------------------
    # Simulation update
    # -----------------------------------------------------------

    def update(self):

        current_time = self.time

        # -------------------------------------------------------
        # Update sensors
        # -------------------------------------------------------

        gps = self.gps.read(
            self.vehicle,
            current_time
        )

        imu = self.imu.read(
            self.vehicle,
            current_time
        )

        camera = self.camera.read(
            self.vehicle,
            self.obstacles,
            current_time
        )

        lidar = self.lidar.read(
            self.vehicle,
            self.obstacles,
            current_time
        )

        wheel = self.wheel.read(
            self.vehicle,
            current_time
        )

        # -------------------------------------------------------
        # Fault detection
        # -------------------------------------------------------

        sensors = [
            self.gps,
            self.imu,
            self.camera,
            self.lidar,
            self.wheel,
            self.brake
        ]

        faults = (
            self.fault_detector.evaluate(
                sensors,
                current_time
            )
        )

        # -------------------------------------------------------
        # Localization
        # -------------------------------------------------------

        self.localization.update(
            gps,
            imu,
            wheel,
            current_time
        )

        # -------------------------------------------------------
        # Safety manager
        # -------------------------------------------------------

        mode = self.safety.evaluate(
            faults,
            self.vehicle.velocity
        )

        # -------------------------------------------------------
        # Controller
        # -------------------------------------------------------

        command = (
            self.controller.calculate(
                self.vehicle,
                self.obstacles,
                camera,
                lidar,
                self.safety,
                current_time
            )
        )

        # -------------------------------------------------------
        # Apply control
        # -------------------------------------------------------

        acceleration = (
            self.apply_control(
                command
            )
        )

        # -------------------------------------------------------
        # Vehicle update
        # -------------------------------------------------------

        self.vehicle.update(
            acceleration
        )

        # -------------------------------------------------------
        # Obstacle movement
        # -------------------------------------------------------

        self.update_obstacles()

        # -------------------------------------------------------
        # Error
        # -------------------------------------------------------

        position_error = math.sqrt(
            (
                self.localization.x
                -
                self.vehicle.x
            ) ** 2
            +
            (
                self.localization.y
                -
                self.vehicle.y
            ) ** 2
        )

        # -------------------------------------------------------
        # Store history
        # -------------------------------------------------------

        self.time_history.append(
            current_time
        )

        self.true_x.append(
            self.vehicle.x
        )

        self.true_y.append(
            self.vehicle.y
        )

        self.estimated_x.append(
            self.localization.x
        )

        self.estimated_y.append(
            self.localization.y
        )

        self.speed_history.append(
            self.vehicle.velocity
        )

        self.confidence_history.append(
            self.localization.confidence
        )

        self.mode_history.append(
            mode
        )

        self.localization_history.append(
            self.localization.source
        )

        self.active_fault_history.append(
            list(faults)
        )

        self.position_error.append(
            position_error
        )

        self.gps_available_history.append(
            self.gps.available
        )

        self.imu_available_history.append(
            self.imu.available
        )

        self.camera_available_history.append(
            self.camera.available
        )

        self.lidar_available_history.append(
            self.lidar.available
        )

        self.wheel_available_history.append(
            self.wheel.available
        )

        self.brake_available_history.append(
            self.brake.available
        )

        self.time += DT


# ===============================================================
# VISUALIZATION
# ===============================================================

class AutonomousDashboard:

    def __init__(
        self,
        simulation
    ):

        self.sim = simulation

        self.fig = plt.figure(
            figsize=(18, 10)
        )

        # -------------------------------------------------------
        # Main road
        # -------------------------------------------------------

        self.ax = self.fig.add_axes(
            [
                0.04,
                0.48,
                0.67,
                0.46
            ]
        )

        # -------------------------------------------------------
        # Fault timeline
        # -------------------------------------------------------

        self.fault_ax = self.fig.add_axes(
            [
                0.04,
                0.08,
                0.67,
                0.30
            ]
        )

        # -------------------------------------------------------
        # Dashboard
        # -------------------------------------------------------

        self.dashboard = self.fig.add_axes(
            [
                0.74,
                0.06,
                0.24,
                0.88
            ]
        )

        # -------------------------------------------------------
        # Road
        # -------------------------------------------------------

        self.ax.set_xlim(
            0,
            ROAD_LENGTH
        )

        self.ax.set_ylim(
            0,
            ROAD_WIDTH
        )

        self.ax.set_title(
            "Autonomous Vehicle Fault Detection & Fail-Safe Control",
            fontsize=16,
            fontweight="bold"
        )

        self.ax.set_xlabel(
            "Road Position (m)"
        )

        self.ax.set_ylabel(
            "Lane Position (m)"
        )

        self.ax.grid(
            alpha=0.2
        )

        # -------------------------------------------------------
        # Lane markings
        # -------------------------------------------------------

        for y in [
            LANE_WIDTH,
            2 * LANE_WIDTH
        ]:

            self.ax.axhline(
                y,
                linestyle="--",
                linewidth=1.2,
                alpha=0.7
            )

        # -------------------------------------------------------
        # Vehicle
        # -------------------------------------------------------

        self.vehicle_patch = Rectangle(
            (
                self.sim.vehicle.x
                -
                VEHICLE_LENGTH / 2,
                self.sim.vehicle.y
                -
                VEHICLE_WIDTH / 2
            ),
            VEHICLE_LENGTH,
            VEHICLE_WIDTH,
            fill=False,
            linewidth=3
        )

        self.ax.add_patch(
            self.vehicle_patch
        )

        # -------------------------------------------------------
        # Estimated vehicle
        # -------------------------------------------------------

        self.estimated_vehicle = Rectangle(
            (
                self.sim.localization.x
                -
                VEHICLE_LENGTH / 2,
                self.sim.localization.y
                -
                VEHICLE_WIDTH / 2
            ),
            VEHICLE_LENGTH,
            VEHICLE_WIDTH,
            fill=False,
            linestyle="--",
            linewidth=2
        )

        self.ax.add_patch(
            self.estimated_vehicle
        )

        # -------------------------------------------------------
        # True trajectory
        # -------------------------------------------------------

        self.true_line, = self.ax.plot(
            [],
            [],
            linewidth=2.5,
            label="True Vehicle Path"
        )

        # -------------------------------------------------------
        # Estimated trajectory
        # -------------------------------------------------------

        self.estimated_line, = (
            self.ax.plot(
                [],
                [],
                linestyle="--",
                linewidth=2,
                label="Estimated Path"
            )
        )

        # -------------------------------------------------------
        # Obstacles
        # -------------------------------------------------------

        self.obstacle_patches = []

        for obstacle in self.sim.obstacles:

            patch = Circle(
                (
                    obstacle["x"],
                    obstacle["y"]
                ),
                2.0,
                fill=True,
                alpha=0.75
            )

            self.ax.add_patch(
                patch
            )

            self.obstacle_patches.append(
                patch
            )

        # -------------------------------------------------------
        # State text
        # -------------------------------------------------------

        self.state_text = self.ax.text(
            5,
            16.5,
            "",
            fontsize=11,
            fontweight="bold",
            bbox=dict(
                boxstyle="round",
                alpha=0.85
            )
        )

        self.ax.legend(
            loc="upper left"
        )

        # =======================================================
        # FAULT TIMELINE
        # =======================================================

        self.fault_ax.set_xlim(
            0,
            SIMULATION_TIME
        )

        self.fault_ax.set_ylim(
            0,
            7
        )

        self.fault_ax.set_xlabel(
            "Simulation Time (s)"
        )

        self.fault_ax.set_title(
            "Sensor Fault / Recovery Timeline"
        )

        sensor_names = [
            "GPS",
            "IMU",
            "CAMERA",
            "LIDAR",
            "WHEEL",
            "BRAKE"
        ]

        self.fault_ax.set_yticks(
            range(1, 7)
        )

        self.fault_ax.set_yticklabels(
            sensor_names
        )

        self.fault_ax.grid(
            alpha=0.2
        )

        # Fault regions

        self.fault_regions = []

        for index, sensor in enumerate(
            sensor_names
        ):

            start, end = (
                FAULT_WINDOWS[sensor]
            )

            region = self.fault_ax.barh(
                index + 1,
                end - start,
                left=start,
                height=0.65,
                alpha=0.25
            )

            self.fault_regions.append(
                region
            )

        # Current time

        self.time_marker = (
            self.fault_ax.axvline(
                0,
                linewidth=2
            )
        )

        # =======================================================
        # DASHBOARD
        # =======================================================

        self.dashboard.axis(
            "off"
        )

        self.dashboard_text = (
            self.dashboard.text(
                0.02,
                0.98,
                "",
                verticalalignment="top",
                fontsize=9.5,
                family="monospace"
            )
        )

    # -----------------------------------------------------------
    # Animation update
    # -----------------------------------------------------------

    def update(
        self,
        frame
    ):

        self.sim.update()

        sim = self.sim

        vehicle = sim.vehicle

        localization = sim.localization

        # -------------------------------------------------------
        # Vehicle patch
        # -------------------------------------------------------

        self.vehicle_patch.set_xy(
            (
                vehicle.x
                -
                VEHICLE_LENGTH / 2,
                vehicle.y
                -
                VEHICLE_WIDTH / 2
            )
        )

        # -------------------------------------------------------
        # Estimated vehicle
        # -------------------------------------------------------

        self.estimated_vehicle.set_xy(
            (
                localization.x
                -
                VEHICLE_LENGTH / 2,
                localization.y
                -
                VEHICLE_WIDTH / 2
            )
        )

        # -------------------------------------------------------
        # Trajectories
        # -------------------------------------------------------

        self.true_line.set_data(
            sim.true_x,
            sim.true_y
        )

        self.estimated_line.set_data(
            sim.estimated_x,
            sim.estimated_y
        )

        # -------------------------------------------------------
        # Obstacles
        # -------------------------------------------------------

        for patch, obstacle in zip(
            self.obstacle_patches,
            sim.obstacles
        ):

            patch.center = (
                obstacle["x"],
                obstacle["y"]
            )

        # -------------------------------------------------------
        # Fault status
        # -------------------------------------------------------

        faults = (
            sim.fault_detector.active_faults
        )

        if faults:

            fault_text = (
                ", ".join(faults)
            )

        else:

            fault_text = "NONE"

        # -------------------------------------------------------
        # State
        # -------------------------------------------------------

        state = sim.safety.mode

        # -------------------------------------------------------
        # Main status
        # -------------------------------------------------------

        self.state_text.set_text(
            f"STATE: {state}\n"
            f"ACTIVE FAULTS: {fault_text}"
        )

        # -------------------------------------------------------
        # Timeline current position
        # -------------------------------------------------------

        self.time_marker.set_xdata(
            [sim.time, sim.time]
        )

        # -------------------------------------------------------
        # Sensor status
        # -------------------------------------------------------

        gps_status = (
            "ONLINE"
            if sim.gps.available
            else "FAILED"
        )

        imu_status = (
            "ONLINE"
            if sim.imu.available
            else "FAILED"
        )

        camera_status = (
            "ONLINE"
            if sim.camera.available
            else "FAILED"
        )

        lidar_status = (
            "ONLINE"
            if sim.lidar.available
            else "FAILED"
        )

        wheel_status = (
            "ONLINE"
            if sim.wheel.available
            else "FAILED"
        )

        brake_status = (
            "ONLINE"
            if sim.brake.available
            else "FAILED"
        )

        # -------------------------------------------------------
        # Position error
        # -------------------------------------------------------

        error = (
            sim.position_error[-1]
            if sim.position_error
            else 0
        )

        # -------------------------------------------------------
        # Dashboard
        # -------------------------------------------------------

        dashboard_text = f"""
╔══════════════════════════════╗
║ AUTONOMOUS SAFETY CONTROLLER ║
╚══════════════════════════════╝

TIME
{sim.time:8.2f} s

VEHICLE STATE
{state}

VELOCITY
{vehicle.velocity:8.2f} m/s

ACCELERATION
{vehicle.acceleration:8.2f} m/s²

──────────────────────────────

TRUE POSITION

X = {vehicle.x:8.2f} m
Y = {vehicle.y:8.2f} m

ESTIMATED POSITION

X = {localization.x:8.2f} m
Y = {localization.y:8.2f} m

POSITION ERROR
{error:8.2f} m

──────────────────────────────

LOCALIZATION SOURCE

{localization.source}

CONFIDENCE
{localization.confidence * 100:6.1f} %

──────────────────────────────

SENSOR HEALTH

GPS
  {gps_status}

IMU
  {imu_status}

CAMERA
  {camera_status}

LIDAR
  {lidar_status}

WHEEL ENCODER
  {wheel_status}

BRAKE
  {brake_status}

──────────────────────────────

FAULT DETECTION

ACTIVE:
{fault_text}

FAULT COUNT
{len(faults)}

──────────────────────────────

FAIL-SAFE LOGIC

BRAKE AVAILABLE
{"YES" if sim.brake.available else "NO"}

EMERGENCY BRAKE
{"ACTIVE" if sim.safety.emergency_braking else "OFF"}

STEERING
{"AVAILABLE" if sim.safety.steering_available else "LIMITED"}

──────────────────────────────

AUTONOMY LEVEL

{state}

"""

        self.dashboard_text.set_text(
            dashboard_text
        )

        return [
            self.vehicle_patch,
            self.estimated_vehicle,
            self.true_line,
            self.estimated_line,
            self.state_text,
            self.dashboard_text,
            self.time_marker
        ] + self.obstacle_patches


# ===============================================================
# MAIN
# ===============================================================

def main():

    random.seed(42)

    np.random.seed(42)

    simulation = (
        AutonomousSimulation()
    )

    dashboard = (
        AutonomousDashboard(
            simulation
        )
    )

    # -----------------------------------------------------------
    # IMPORTANT:
    #
    # Keep the FuncAnimation object stored in a variable.
    # This prevents Matplotlib from garbage-collecting the
    # animation before plt.show().
    # -----------------------------------------------------------

    animation = FuncAnimation(
        dashboard.fig,
        dashboard.update,
        frames=None,
        interval=DT * 1000,
        blit=False,
        cache_frame_data=False
    )

    dashboard.animation = animation

    plt.show()


# ===============================================================
# ENTRY POINT
# ===============================================================

if __name__ == "__main__":

    main()