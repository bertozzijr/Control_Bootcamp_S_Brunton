# Control_Bootcamp_S_Brunton
Matlab Code by Steven Brunton on his Control Bootcamp Lessons

UTILITIES

cartpend.m           — system of ODEs for inverted pendulum on a cart

drawcartpend.m       — draw inverted pendulum on cart

drawcartpend_bw.m    — draw with inverted colors

PRELIMINARIES

linearize_cartpend.m — test linearized dynamics

sim_cartpend.m       — simulate nonlinear dynamics

CONTROL AND ESTIMATOR DESIGN

poleplace_cartpend.m — pole placement for controllable system

lqr_cartpend.m       — design LQR full-state feedback controller

obsv_cartpend.m      - test observability

kf_cartpend.m        - design Kalman filter for full-state estimation

Simulink implementations of LQR and LQG in folder “simulink_LQR_LQG”

lqg_simulinkINIT.m   — initializes LQR feedback gains and Kalman filter system

lqr_cartpend_sim.slx — simulink model for LQR

lqg_cartpend_sim.slx — simulink model for LQG

cartpend_sim.*       — system block for inverted pendulum on cart
