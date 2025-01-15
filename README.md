# Flight Path Tracking Project
확장 칼만 필터(EKF)를 이용한 궤적 추정 알고리즘을 구현, 로켓이 비행하는 동안 얻은 IMU, GPS, 기압 센서 데이터를 종합하여 처리하고 비행 궤적을 계산하는 하나로 전자팀 산하 프로젝트입니다.

This project processes and visualizes IMU (Inertial Measurement Unit), GPS, and pressure sensor data from a flight trajectory, implementing various trajectory estimation methods including Extended Kalman Filter (EKF).

## Project Structure
```mermaid
graph TD
    subgraph Main Flow
        A[main.py] --> B[data.py: data_format]
        B --> C[Set Initial Conditions]
        C --> D[Extended Kalman Filter]
        D --> E[Plot Trajectories]
    end

    subgraph Data Processing
        B --> F[Read CSV Data]
        F --> G[Process IMU Data]
        F --> H[Process GPS Data]
        F --> I[Process Pressure Data]
    end

    subgraph Trajectory Classes
        J[IMUTrajectory] --> K[Calculate IMU-based Path]
        L[GPSTrajectory] --> M[Calculate GPS-based Path]
        N[SimpleTrajectory] --> O[Calculate Simple Path]
    end

    subgraph Configuration
        P[config.py] --> Q[Flight States]
        P --> R[Plot Settings]
    end

    E --> J
    E --> L
    E --> N
    Q --> B
    R --> E
```

- Main Flow(`main.py`): 
  - Entry point of the application
  - Creates a data_format object
  - Initializes conditions and runs EKF (Extended Kalman Filter)
  - Sets up 3D plotting environment
  - Plots three different trajectory calculations
- Data Processing(`data.py`):
  - `data_format` class: Core data processing class
  - Handles sensor data:
    - Accelerometer
    - Gyroscope
    - Magnetometer
    - GPS coordinates
    - Pressure/Altitude data
  - Implements quaternion-based rotation calculations
    - `quaternion.py`: Quaternion mathematics utilities for 3D rotation calculations
  - Provides methods for initial condition setup and EKF processing
- Trajectory Classes(`plot_trajectory.py`): Implements various trajectory estimation methods:
  - `IMUTrajectory`: Calculates trajectory using IMU data
    - Uses RK4 integration
    - Handles acceleration transformation from body to observer frame
  - `GPSTrajectory`: Processes GPS-based trajectory
    - Uses haversine formula for coordinate conversion
  - `SimpleTrajectory`: Calculates trajectory based on simple velocity integration
    - Inherits from IMUTrajectory
    - Uses gyroscope data for rotation
- Configuration(`config.py`): 
  - Stores global configuration
  - Defines flight states:
    - stand_by
    - launch
    - apogee
    - drogue
    - main
    - touchdown
  - Contains plotting parameters
  - Defines physical constants

## Usage
`main.py` 스크립트를 실행하면 IMU, GPS, 그리고 기압 센서 데이터를 기록한 CSV 파일을 처리하여 비행 궤적을 구하고, 다음의 세 가지 방식으로 추정한 경로를 각각 시각화하여 나타냅니다.  
The main script processes data from a CSV file containing IMU, GPS, and pressure sensor readings. It visualizes three different trajectory estimates:
- Kalman filter-based trajectory (red)
- GPS-based trajectory (green)
- Simple integration-based trajectory estimation (blue)

## Mathematical Foundation
### Trajectory Estimation Methods
| Method | Advantages | Limitations |
|--------|------------|-------------|
| IMU Trajectory | - High temporal resolution <br>- Accurate short-term tracking <br>- Independent of external signals | - Drift accumulation <br>- Requires accurate initial conditions |
| GPS Trajectory | - Absolute position reference <br>- No drift over time <br>- Simple implementation | - Lower update rate <br>- Dependent on signal availability |
| Simple Trajectory | - Computationally efficient <br>- Good for rough estimates <br>- Easy to implement | - Less accurate than RK4 <br>- Susceptible to integration errors |

#### IMU Trajectory Estimation
```mermaid
graph TD
    A[IMU Data Input] --> B[Body Frame Acceleration]
    B --> C[Quaternion Rotation]
    C --> D[Observer Frame Acceleration]
    D --> E[RK4 Integration]
    E --> F[Position Update]
    
    subgraph RK4 Steps
        E --> G[k1 calculation]
        E --> H[k2 calculation]
        E --> I[k3 calculation]
        E --> J[k4 calculation]
        G & H & I & J --> K[Combined Update]
    end
```

- Runge-Kutta 4th Order(RK4) integration:
  For position $\vec{r}$ and velocity $\vec{v}$,
  - $\vec{k_1} = f(t_n, \vec{r_n})$
  - $\vec{k_2} = f(t_n + \frac{\Delta t}{2}, \vec{r_n} + \vec{k_1}\cfrac{\Delta t}{2})$
  - $\vec{k_3} = f(t_n + \frac{\Delta t}{2}, \vec{r_n} + \vec{k_2}\cfrac{\Delta t}{2})$
  - $\vec{k_4} = f(t_n + \Delta t, \vec{r_n} + \Delta t\vec{k_3})$
  - Final update: $\vec{r_{n+1}} = \vec{r_n} + \cfrac{\Delta t}{6}(\vec{k_1} + 2\vec{k_2} + 2\vec{k_3} + \vec{k_4})$
- Handles acceleration transformation from body to observer frame
  - $\vec{a}_{observer} = q \otimes \vec{a}_{body} \otimes q^*$ 
  - where: $q = q_0 + q_1i + q_2j + q_3k$

#### GPU Trajectory Estimation
```mermaid
graph TD
    A[GPS Data Input] --> B[Convert to Radians]
    B --> C[Calculate Reference Point]
    C --> D[Haversine Distance]
    D --> E[Local Coordinate Transform]
    E --> F[3D Position Output]
```

- Haversine Formula:
  $d = 2R \arcsin\left(\sqrt{\sin^2\left(\frac{\Delta\phi}{2}\right) + \cos\phi_1\cos\phi_2\sin^2\left(\frac{\Delta\lambda}{2}\right)}\right)$
  - $R$: Earth's radius(지구 반지름)
  - $\phi$: latitude(위도)
  - $\lambda$: longitude(경도)
- Local Coordinate Transform:
  - $x = d\cos(\theta)$
  - $y = d\sin(\theta)$
  - $z = h$ (altitude)

#### Simple Trajectory Estimation
Inherits from IMUTrajectory but uses a simplified integration approach.

```mermaid
graph TD
    A[IMU Data Input] --> B[Gyroscope Integration]
    B --> C[Simple Rotation Matrix]
    C --> D[Acceleration Transform]
    D --> E[Velocity Integration]
    E --> F[Position Update]
```

- Gyroscope Integration: $q_{n+1} = q_n + \frac{\Delta t}{2}q_n \otimes \omega_n$
- Position Update:
  - $\vec{v_{n+1}} = \vec{v_n} + \vec{a_n}\Delta t$
  - $\vec{r_{n+1}} = \vec{r_n} + \vec{v_n}\Delta t + \cfrac{1}{2}\vec{a_n}(\Delta t)^2$

## Dependencies
- pandas
- numpy
- matplotlib
- scipy
