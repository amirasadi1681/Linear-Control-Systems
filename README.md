# Linear Control of a Quadrotor (MATLAB / Simulink)

This project was developed as part of the **Linear Control Systems** course.  
It focuses on modeling, linearizing, and designing SISO controllers for a quadrotor drone around hover conditions using MATLAB and Simulink.

---

## Overview

The goal of this project was to design and implement linear controllers that stabilize and control the attitude and altitude of a quadrotor.  
Starting from the nonlinear equations of motion, the system was linearized around the hover equilibrium point and represented in state-space form.

The resulting model was analyzed in MATLAB and validated through Simulink simulations.

---

## Methodology

1. **System Modeling**  
   - A 12-state model was formulated to represent the translational and rotational dynamics of the quadrotor.  
   - The system was linearized about the hover condition using small-angle approximations.

2. **Controller Design**  
   - Each channel (roll, pitch, yaw, altitude) was treated as a separate SISO system.  
   - PD and PI/PD controllers were designed to achieve desired transient performance (low overshoot, fast settling).  
   - Controllers were tuned both analytically (using phase and root-locus methods) and with MATLAB’s PID Tuner.

3. **Simulation and Validation**  
   - The designed controllers were tested in Simulink on both linear and nonlinear quadrotor models.  
   - Step responses, Bode plots, Nyquist diagrams, and root loci were analyzed to verify stability and performance.  
   - The final model achieved stable flight control and satisfactory transient behavior in all channels.

---

## Simulink Model

The simulation environment was built in Simulink, including:
- Individual **PD/PI controllers** for each input channel  
- A **quadrotor dynamic block** modeling system behavior  
- Scope blocks for visualizing roll, pitch, yaw, and altitude responses  

Below is a simplified view of the simulation setup:

![Simulink Model](docs/figures/simulink_overview.png)

---

## Repository Structure

