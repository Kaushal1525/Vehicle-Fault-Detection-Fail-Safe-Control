# Autonomous Vehicle Fault Detection & Fail-Safe Control

A Python-based autonomous vehicle simulation that models **sensor faults, localization degradation, fault detection, safety-state management, and fail-safe control** in an autonomous driving environment.

The system follows the pipeline:

**Sensors → Localization → Fault Detection → Safety Manager → Autonomous Controller → Vehicle**

## 🚗 Key Features

* GPS, IMU, camera, LiDAR, and wheel encoder simulation
* Brake-system fault simulation
* Configurable sensor fault windows
* Sensor reliability and confidence tracking
* GPS-based localization
* IMU-assisted dead reckoning
* Wheel odometry integration
* Multi-sensor position estimation
* Position estimation error monitoring
* Automatic fault detection
* Sensor health monitoring
* Degraded autonomy mode
* Safe-stop mode
* Critical-fault mode
* Emergency braking
* Brake availability monitoring
* Steering availability monitoring
* Obstacle detection using camera and LiDAR
* Automatic obstacle-distance evaluation
* Dynamic moving obstacles
* Autonomous acceleration and braking control
* True vs estimated vehicle trajectory visualization
* Fault and recovery timeline
* Real-time autonomous safety dashboard
* Deterministic simulation using seeded randomness

## 🧠 System Architecture

```text
                 AUTONOMOUS VEHICLE
                        │
                        ▼
              ┌───────────────────┐
              │      SENSORS      │
              │                   │
              │ GPS   IMU         │
              │ Camera            │
              │ LiDAR             │
              │ Wheel Encoder     │
              │ Brake System      │
              └─────────┬─────────┘
                        │
                        ▼
              ┌───────────────────┐
              │   LOCALIZATION    │
              │                   │
              │ GPS + IMU + Wheel │
              │ Dead Reckoning    │
              │ Position Estimate │
              └─────────┬─────────┘
                        │
                        ▼
              ┌───────────────────┐
              │ FAULT DETECTION   │
              │                   │
              │ Sensor Health     │
              │ Active Faults     │
              │ Confidence        │
              └─────────┬─────────┘
                        │
                        ▼
              ┌───────────────────┐
              │  SAFETY MANAGER   │
              │                   │
              │ Full Autonomy     │
              │ Degraded Autonomy │
              │ Safe Stop         │
              │ Critical Fault    │
              └─────────┬─────────┘
                        │
                        ▼
              ┌───────────────────┐
              │    CONTROLLER     │
              │                   │
              │ Acceleration      │
              │ Braking           │
              │ Emergency Action  │
              └─────────┬─────────┘
                        │
                        ▼
                 VEHICLE DYNAMICS
```

## ⚙️ Simulation Configuration

| Parameter              |    Value |
| ---------------------- | -------: |
| Simulation Time        |     60 s |
| Time Step              |   0.05 s |
| Road Length            |    220 m |
| Road Width             |     18 m |
| Lane Width             |      6 m |
| Initial Speed          |   14 m/s |
| Normal Acceleration    | 1.0 m/s² |
| Normal Deceleration    | 3.5 m/s² |
| Emergency Deceleration | 8.5 m/s² |
| GPS Noise              |    1.8 m |
| IMU Noise              |     0.15 |
| Camera Range           |     45 m |
| LiDAR Range            |     60 m |
| Wheel Noise            |     0.12 |
| Safe Stop Distance     |      5 m |

These simulation parameters and vehicle configuration are defined directly in the project source.

## 🛰️ Sensor Fault Injection

The simulation intentionally introduces faults at different stages to evaluate autonomous-system resilience.

| Sensor/System | Fault Window |
| ------------- | -----------: |
| GPS           |      10–17 s |
| IMU           |      19–25 s |
| Camera        |      28–35 s |
| LiDAR         |      38–45 s |
| Wheel Encoder |      47–53 s |
| Brake         |      55–60 s |

This allows the vehicle to experience progressively different failure conditions rather than operating only in an ideal sensor environment.

## 📡 Sensor Simulation

### GPS

The GPS sensor generates noisy position measurements and becomes unavailable during its configured fault window.

### IMU

The IMU provides acceleration and heading measurements while introducing noise and a continuously changing bias.

### Camera

The camera detects obstacles within a configurable detection range.

### LiDAR

LiDAR provides obstacle detections over a larger configurable range.

### Wheel Encoder

The wheel encoder estimates traveled distance and velocity while incorporating measurement noise.

### Brake System

The brake system independently models brake availability and determines whether requested deceleration can actually be applied.

## 📍 Sensor Fusion & Localization

The `LocalizationSystem` combines available GPS, IMU, and wheel-encoder information.

When GPS measurements are available, position is estimated from the available GPS measurements. When GPS is unavailable, the system switches to dead reckoning using estimated velocity and heading.

The system identifies the current localization source:

```text
GPS + IMU + WHEEL
GPS + IMU
IMU + WHEEL
GPS ONLY
IMU DEAD RECKONING
WHEEL ODOMETRY
NO LOCALIZATION
```

A corresponding confidence value is maintained for each localization state.

## 🛠️ Fault Detection

The `FaultDetectionSystem` continuously checks sensor availability and records active faults.

It tracks:

* Active sensor failures
* Fault history
* Last detected fault
* Sensor confidence
* Overall system confidence

System confidence is calculated from the individual sensor confidence values.

## 🛡️ Fail-Safe Safety Manager

The `SafetyManager` determines the vehicle's operating state according to detected faults.

### Full Autonomy

Normal sensor availability and autonomous operation.

### Degraded Autonomy

Used when limited sensor failures are detected while sufficient functionality remains available.

### Safe Stop

Triggered by severe navigation-sensor degradation, including multiple critical sensor failures or simultaneous IMU and wheel failure.

### Critical Fault

Triggered when the brake system fails.

The safety manager also tracks:

* Emergency braking state
* Brake availability
* Steering availability
* Critical-fault state
* Safety-mode history

## 🚨 Autonomous Safety Controller

The controller evaluates:

* Current vehicle state
* Camera detections
* LiDAR detections
* Obstacle distance
* Safety mode
* Brake availability

The controller applies different responses depending on obstacle proximity:

```text
Obstacle Distance
       │
       ├── < 8 m  → Emergency Braking
       │
       ├── < 15 m → Strong Braking
       │
       ├── < 25 m → Moderate Braking
       │
       └── > 25 m → Normal Driving
```

If the brake system is unavailable, the controller cannot apply the requested braking command.

## 🚘 Dynamic Obstacles

The simulation contains multiple moving obstacles positioned across the three-lane road.

Each obstacle has:

```text
Position (x, y)
Speed
```

Obstacles continuously move through the environment and are recycled ahead of the vehicle after leaving the simulated road.

## 📊 Real-Time Dashboard

The Matplotlib dashboard provides a live view of the autonomous system.

### Main Road View

Displays:

* True vehicle
* Estimated vehicle
* True trajectory
* Estimated trajectory
* Moving obstacles
* Current safety state
* Active faults

### Fault Timeline

Displays the fault/recovery windows for:

```text
GPS
IMU
CAMERA
LIDAR
WHEEL
BRAKE
```

### Safety Dashboard

Displays:

* Simulation time
* Vehicle state
* Velocity
* Acceleration
* True position
* Estimated position
* Position error
* Localization source
* Localization confidence
* Sensor health
* Active faults
* Fault count
* Brake availability
* Emergency-braking status
* Steering availability
* Current autonomy level

## 🔄 Autonomous Decision Flow

```text
Sensor Measurements
        ↓
Sensor Health Check
        ↓
Fault Detection
        ↓
Localization Update
        ↓
Safety-State Evaluation
        ↓
Obstacle Detection
        ↓
Safety Decision
        ↓
Acceleration / Braking Command
        ↓
Vehicle Update
        ↓
State & Fault Logging
        ↓
Dashboard Visualization
```

The simulation executes this sequence continuously at the configured time step.

## 🛠️ Technologies

* Python
* NumPy
* Matplotlib
* Matplotlib Animation
* Object-Oriented Programming
* Sensor Simulation
* Sensor Fusion
* Fault Detection
* Autonomous Control
* Fail-Safe Logic
* Vehicle Dynamics
* Real-Time Visualization

## 📦 Installation

Install the required Python packages:

```bash
pip install numpy matplotlib
```

## ▶️ Run

Save the program as:

```text
autonomous_fault_detection.py
```

Then execute:

```bash
python autonomous_fault_detection.py
```

The simulation launches the animated autonomous-vehicle dashboard.

## 🎯 Autonomous Vehicle Applications

This simulation demonstrates concepts relevant to:

* Autonomous driving
* ADAS
* Sensor redundancy
* Fault-tolerant autonomy
* Vehicle localization
* Sensor-health monitoring
* Fail-operational / fail-safe strategies
* Emergency braking
* Autonomous safety systems
* Sensor degradation management
* Safety-state transitions
* Autonomous vehicle testing

## 🔬 Future Development

Possible extensions include:

* Extended Kalman Filter localization
* Radar integration
* Multi-object tracking
* Probabilistic sensor fusion
* Realistic camera perception
* LiDAR point-cloud processing
* Vehicle CAN-bus simulation
* Redundant brake controllers
* Steering fault modeling
* MPC-based emergency control
* ROS 2 integration
* CARLA simulation
* Hardware-in-the-loop testing
* Real vehicle sensor datasets
* ISO 26262-oriented safety analysis
* ASIL-oriented fault scenarios

## 👨‍💻 Author

**Kaushal Jammula**

Graduate | Former Vice President @Aprameya | Entrepreneur | Focused - Automotive Systems Specialist | Space Tech Enthusiast | Researcher | Emerging Tech Innovator | Engineering Beyond Limits

GitHub: **Kaushal1525**

## 📌 Project

**Autonomous Vehicle Fault Detection & Fail-Safe Control**

A simulation-oriented project exploring how an autonomous vehicle can **detect sensor failures, estimate its state with available information, transition between safety modes, and respond with appropriate fail-safe control behavior.**
