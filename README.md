# 🤖 6 DOF Robot Arm - Embedded System Project
This project implements a 6 Degrees if Freedom (6 DoF) robotic arm , controlled by an STM32 microcontroller. The system communicates with a PC via UART and includes a Python-based GUI for manual control or using YOLO to detect object to create trajectory for auto control 

## 📌 Objectives
  - Control 6 step motors corresponding to 6 joints of the robotic arm
  - Communicate with a PC via UART (Serial communication)
  - Provide a real-time GUI to control and visualize joint positions
  - Deliver comple 3D designs and PCB schematics 

## 🛠️ Technologies Used
| Component        | Tool / Technology         |
|------------------|---------------------------|
| Microcontroller  | STM32F411CEU6             |
| Mechanical frame | 3D printed parts          |
| IDE              | Keil µVision              |
| Communication    | UART (via USB-TTL/RS232)  |
| PC Interface     | Python (Tkinter GUI)      |
| CAD Modeling     | SolidWorks                |
| PCB Design       | Altium Designer           |
| Simulation       | CoppeliaSim               |

## 📁 Project Structure

| Folder/File         | Description                                                                 |
|---------------------|-----------------------------------------------------------------------------|
| `Hardware/`          | Contains all mechanical and electrical design files.                       |
| └── `CAD/`           | 3D models (STEP/STL) for 3D printing and simulation.                       |
| └── `Schematics/`    | Circuit diagrams and wiring documents.                                     |
| `Software/`          | Source code for firmware and desktop control.                              |
| └── `KeilC/`         | STM32 firmware developed with Keil µVision.                                |
| └── `Python/`        | Python scripts for PC control, visualization, and GUI.                     |
| `images/`            | Screenshots and images for documentation and README.                       |
| `.gitignore`         | List of files/folders ignored by Git.                                      |
| `README.md`          | Project overview (this file).  

## 📸 Demo
![video demo](https://raw.githubusercontent.com/hoanganhdo207/ARM_6_DoF/blob/main/Images/Thumbnail.png)(https://drive.google.com/file/d/13h31Oaqvu5-RNBlb9Kudcmn5dsyJSfwP/view?usp=sharing) 
