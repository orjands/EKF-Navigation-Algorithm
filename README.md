# Remora Rover Navigation – Master Thesis 2025

This repository contains the complete code developed as part of Ørjan D. Strand’s master’s thesis (Spring 2025), focused on model-based navigation for Remora Robotics’ underwater net-grooming robot, the **Remora Rover**. The goal was to develop and evaluate an Extended Kalman Filter (EKF)-based navigation system capable of operating in dynamic and sensor-noisy aquaculture environments.

## 🚀 Project Overview

- **Title**: Autonomous Navigation of an Underwater Net-Grooming Robot: Combining Dynamic Modelling and Inertial Sensing
- **Institution**: NTNU – Norwegian University of Science and Technology
- **Supervisor**: Amund Skavhaug
- **Co-supervisor**: Christian Holden
- **Collaborator**: Remora Robotics
- **Thesis period**: January–June 2025

## 🔧 Requirements
- MATLAB R2021b or later
- [EKF/MATH/MAGNETOMETER.m](EKF/MATH/MAGNETOMETER.m) — Requires Symbolic Math Toolbox
- [WMM2025COF/WMM.COF](WMM2025COF/WMM.COF) — Has to be added to path for the EKF to work


## 📊 Features
- Full MATLAB navigation simulator with:
  - Net deformation
  - Ocean currents
  - Model-based EKF algorithm
  - Sensor noise and model uncertainty
- Real-world test processing from Remora Rover field data
- Test rig validation for noise-free evaluation

## 📜 How to Use

1. Clone the repository:
   ```bash
   git clone https://github.com/<your-username>/remora-rover-navigation.git
   cd EKF-Navigation-Algorithm
2. Open MATLAB
3. Add the folder "EKF-Navigation-Algorithm" to path by either:

   3.1 Using the MATLAB GUI:  
   - Open MATLAB  
   - Right-click the folder in the File Browser  
   - Click *"Add to Path" > "Selected Folders and Subfolders"*

   3.2 Using the MATLAB command window:  
   ```matlab
   addpath(genpath('path_to_your_repo'));)
   savepath
4. Navigate to the folder relevant to the test or simulation you wish to run.
5. Play around and have fun! 🤓
