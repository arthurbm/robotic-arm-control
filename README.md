# Robotic Arm Control Projects
### IF826 - Robotics (Sistemas Inteligentes - Robótica)
#### CIn-UFPE

This repository contains the projects developed for the Robotics course at CIn-UFPE. The projects focus on implementing and controlling robotic systems, with emphasis on 2D robotic arm manipulation.

## Project Structure

The repository is organized into projects, each containing one or more questions:

```
.
├── project1/
│   ├── q1/                    # Question 1: 2D Transformations
│   │   └── main.py           # SE2 transformations implementation
│   └── q2/                    # Question 2: Robotic Arm
│       └── main.py           # FK/IK implementation
├── env_project/               # Virtual environment (not tracked)
├── .gitignore
└── README.md
```

### Project 1: 2D Robotic Arm Control

#### Question 1: 2D Homogeneous Transformations
Located in `project1/q1/`, this implementation includes:
- SE(2) transformations for translation and rotation
- Functions for coordinate frame transformations
- Test cases for different reference frame scenarios

#### Question 2: Planar Robotic Arm with Two Rotational Joints
Located in `project1/q2/`, this implementation includes:
- Forward Kinematics (FK) implementation
- Inverse Kinematics (IK) implementation
- Simulation with link lengths a1 = a2 = 1m

### Project 2: TBD
Details will be added when the second project is assigned.

## Dependencies

The project uses minimal dependencies to focus on fundamental implementations:
- Python 3.x
- NumPy
- Math (Python standard library)

## Setup and Installation

### 1. Clone the repository:
```bash
git clone [repository-url]
cd robotic-arm-control
```

### 2. Set up Virtual Environment

#### On Linux/macOS:
```bash
# Install venv if not already installed
sudo apt-get install python3-venv  # For Ubuntu/Debian

# Create virtual environment
python3 -m venv env_project

# Activate virtual environment
source env_project/bin/activate
```

#### On Windows:
```powershell
# Create virtual environment
python -m venv env_project

# Activate virtual environment
.\env_project\Scripts\activate
```

### 3. Install Dependencies
With the virtual environment activated (you should see `(env_project)` in your terminal):
```bash
pip install numpy
```

### 4. Deactivating the Virtual Environment
When you're done working on the project:
```bash
deactivate
```

## Usage

With the virtual environment activated, you can run each question's implementation:

```bash
# For Question 1 (2D Transformations)
cd project1/q1
python main.py

# For Question 2 (Robotic Arm)
cd project1/q2
python main.py
```

## Implementation Details

### Question 1: 2D Transformations

1. **SE2_xy(x, y)**
   - Creates a 2D homogeneous transformation matrix for translation
   - Parameters:
     - x: translation along x-axis (meters)
     - y: translation along y-axis (meters)
   - Returns: 3x3 homogeneous transformation matrix

2. **SE2_theta(theta)**
   - Creates a 2D homogeneous transformation matrix for rotation
   - Parameters:
     - theta: rotation angle around origin (radians)
   - Returns: 3x3 homogeneous transformation matrix

### Question 2: Robotic Arm

1. **Forward Kinematics (fk)**
   - Calculates end-effector position and orientation
   - Parameters:
     - theta1: first joint angle (radians)
     - theta2: second joint angle (radians)
   - Returns: (x, y, theta) coordinates and orientation

2. **Inverse Kinematics (ik)**
   - Calculates joint angles for desired end-effector position
   - Parameters:
     - x: desired x-coordinate (meters)
     - y: desired y-coordinate (meters)
   - Returns: (theta1, theta2) joint angles

## Common Issues and Solutions

1. **Virtual Environment Not Activating (Windows)**
   - If you get a permission error, try running PowerShell as administrator
   - Make sure execution policy allows running scripts: `Set-ExecutionPolicy RemoteSigned -Scope CurrentUser`

2. **Virtual Environment Not Found**
   - Ensure you're in the project root directory when creating/activating the environment
   - Check if Python is properly installed and added to PATH

## Authors

- Student: [Your Name]
- Course: IF826 - Robotics
- Institution: Centro de Informática, Universidade Federal de Pernambuco (CIn-UFPE)
- Year: 2024 