# **6-DOF Industrial Robot Arm: Digital Twin & Kinematic Analysis**

This project covers the full engineering design and validation pipeline for a 6-Axis (6-DOF) Industrial Robotic Arm. It connects 3D CAD modeling, ROS 2 digital twin setup, MATLAB physical simulation, mathematical forward kinematics analysis, and 2D manufacturing blueprints into a single unified workflow.

---

## **📌 Project Overview**

Most robotics projects focus either only on 3D CAD or only on software code. This project bridges both worlds by creating a physically accurate **Digital Twin** that links:

1. **Mechanical Design (Fusion 360)**: Parametric 3D CAD assembly, component hierarchies, revolute joint limits, and physical material assignments (Aluminum 6061 / Steel).
2. **ROS 2 Export (URDF)**: Automatic export of Unified Robot Description Format (`.urdf`) files and 3D STL collision/visual mesh files.
3. **Physical Simulation (MATLAB Simscape Multibody)**: Converting URDF models into physical block diagrams with link inertia and joint constraints.
4. **Kinematic Analysis (MATLAB Code)**: Computing 3D Cartesian coordinates ($X, Y, Z$) for multiple joint configurations using transformation matrices (`getTransform`).
5. **Manufacturing Documentation (AutoCAD / Fusion Drawing)**: 2D production blueprints featuring exploded views, balloon callouts, a Bill of Materials (BOM).

---

## **🛠️ Software & Tools Required**

* **Autodesk Fusion 360**: 3D Parametric CAD, Joints, Motion Study, and 2D Drawings.
* **Fusion_URDF_Exporter_ROS2 Plugin**: Open-source add-in for URDF and STL mesh package generation.
* **MATLAB & Simulink**: R2022b or newer with Simscape Multibody and Robotics System Toolbox.


---

## **🚀 Step-by-Step Build Process**

### **Step 1: 3D CAD Modeling & Assembly (Fusion 360)**

1. **Component Hierarchy**: Organized the assembly tree so every moving link is an unnested top-level **Component** (e.g., `base_link`, `shoulder`, `elbow`, `wrist`, `gripper`).
2. **Physical Materials**: Assigned real engineering materials (Steel) to automatically calculate exact mass, center of mass, and moment of inertia matrices ($I_{xx}, I_{yy}, I_{zz}$).
3. **Revolute Joints & Limits**: Set up 8 independent revolute joints ($J_1$ through $J_8$) with physical angular rotation boundaries to prevent link self-collisions.

---

### **Step 2: URDF & STL Mesh Export**

1. Grounded the root component and verified the top-level link was named strictly `base_link`.
2. Ran the open-source `Fusion_URDF_Exporter_ROS2` add-in (`Shift + S`).
3. Generated a complete ROS 2 description package containing:

* `RobotArm.urdf`: XML description of links, joint axes, inertial parameters, and visual/collision tags.
* `meshes/`: 3D STL mesh files for each individual arm link and gripper jaw.

---

### **Step 3: MATLAB Simscape Multibody Import**

1. Set the MATLAB working directory to the folder containing `RobotArm.urdf` and the `meshes/` folder.
2. Executed the import command:

```matlab
smimport('RobotArm.urdf')
```

3. MATLAB automatically mapped the URDF links into **Subsystem blocks** (Inertia, File Solid, Rigid Transforms) and revolute joints into **Simscape Joint blocks**.
4. Pressed `Ctrl + D` to compile the model, opening the **Simscape Multibody Explorer** for 3D physical animation.

---

### **Step 4: 6-DOF Kinematic Analysis**

To analyze the relationship between the robot's joint configurations and its end-effector position, a MATLAB script (`fk_6dof_validation.m`) computes the end-effector ($X, Y, Z$) position across 4 operational joint poses:


### **Step 5: 2D Production Drawing & Manufacturing Documentation**

1. Switched to Fusion 360's **Drawing** workspace.
2. Created an **Isometric Exploded View** with numbered balloon callouts for assembly instructions.
3. Placed an automated **Bill of Materials (BOM)** table listing item numbers, part quantities, descriptions, and assigned materials.
4. Added 2D orthographic projection views with primary dimensions, Datum A/B references, and basic ASME GD&T position tolerances.

---




## **💻 How to Run the MATLAB Kinematic Analysis Script**

1. Clone or download this repository:

```bash
git clone https://github.com/ParasBadhran102308152/6-DOF-Industrial-Robot-Arm.git
```

2. Open MATLAB and navigate to the `MATLAB/` folder.
3. Run the analysis script in the Command Window:

```matlab
run('fk_6dof_validation.m')
```

4. To open the 3D Simscape Multibody model:

```matlab
smimport('RobotArm.urdf')
```

Press `Ctrl + D` to update the model and launch the Simscape 3D Multibody Explorer.

---

## **📊 Summary of Engineering Outcomes**

* **CAD Design**: Built a functional 6-DOF industrial arm with proper mechanical joint limits and physical material specifications.
* **Digital Twin**: Created a URDF description package and STL collision meshes directly from CAD.
* **Production Blueprints**: Generated 2D  drawings with a Bill of Materials (BOM).
