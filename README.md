# IK_ROBOT ASSIGNMENT

## About

This repository contains resources for implementing inverse kinematics (IK) for a robotic system. Specifically, it focuses on utilizing Pinocchio, a powerful framework for robot kinematics and dynamics computation. The system pertains to a 6-degree-of-freedom (6DoF) robotic arm designed for operations within a 1-meter hemisphere.

## URDF

### Image



| <img src="media/urdf.gif">             | 
| :----------------------------------: | 
|          _URDF_           | 
## How to Run

### Dependencies

Before running the provided scripts, ensure you have the necessary dependencies installed. Follow these steps:

1. **Add robotpkg apt repository:**

   First, make sure you have some required installation dependencies:
   ```bash
   sudo apt install -qqy lsb-release curl
   ```

   Next, register the authentication certificate of robotpkg:
   ```bash
   sudo mkdir -p /etc/apt/keyrings
   curl http://robotpkg.openrobots.org/packages/debian/robotpkg.asc \
       | sudo tee /etc/apt/keyrings/robotpkg.asc
   ```

   Add robotpkg as a source repository to apt:
   ```bash
   echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" \
       | sudo tee /etc/apt/sources.list.d/robotpkg.list
   ```

   Run at least once `apt update` to fetch the package descriptions:
   ```bash
   sudo apt update
   ```

2. **Install Pinocchio:**

   ```bash
   sudo apt install -qqy robotpkg-py3*-pinocchio
   ```

### Configure Environment Variables

After installing the dependencies, configure your environment variables to point to the installed packages. Add the following lines to your `~/.bashrc` for persistent configuration:

```bash
export PATH=/opt/openrobots/bin:$PATH
export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH
export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH
export PYTHONPATH=/opt/openrobots/lib/python3.10/site-packages:$PYTHONPATH # Adapt your desired Python version here
export CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH
```

### Run

To execute the IK script, use the following command:

```bash
python3 ik.py
```

## Sample Output

![Alt Text](media/sample_output.jpg)

