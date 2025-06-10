
# Soft Constrained Warm Start based MPC-PD Approach for Real Time Control of Underactuated Systems.
Submission ID: 9660

Authors: Gayathri S Menon, Vinopraba T, Lithika J, Kannan S.

All authors are affiliated to: National Institute of Technology, Puducherry.

This document has two SIMULINK files: conventional MPC model code (MPC.slx) and  warm-started MPC-PD Mmodel code (Warmstart_PD_MPC.slx). The hardware setup used is the Quanser Qube Servo-2 pendulum platform. 
In MPc.slx, the function Hardware_run.m has been developed to be directly used during the hardware interface.
In Warmstart_PD_MPC.slx, the MPC function Hardware_fn.m has been developed to be directly used during the hardware interface. The document Warmstart_PD_MPC.slx works based on the following algorithm -
<div align="center">
  <img src="https://github.com/user-attachments/assets/3dca5274-82f3-4d94-8a14-e01529a319a6" width="400">
</div>

This repository contains all files required to reproduce the hardware results presented in the article.

Requirements: Matlab 2021a, Quanser Qube Servo-2 pendulum platform
THe results in Fig. 5-7 and Table II can be reproduced using these codes. Fig. 5. gives the arm trajectory tracking by MPC (conventional) and MPCPD (warm start based) technique, Fig. 6. gives the coontrol signal generated using MPC (conventional) and MPC-PD (warm start based) techniques. Fig. 7. Pendulum angle tracking by MPC (conventional) and MPCPD (warm start based) techniques

For replication of results, contact: EE20D1006@nitpy.ac.in




