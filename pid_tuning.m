#clear out past variables
clc
# load the control package
pkg load control

# Car cruise control model
m=2000; #Mass of car in Kg
b=1; #Viscous friction coefficient
P_cc = 1/(m*s + b); #Cruise Control Transfer function

# PID transfer function setup with initial guess for cruise control
Kp_cc=800;
Ki_cc=40;

s_cc=tf('s');
C_cc=pid(Kp_cc,Ki_cc);

# Setpoints
sp_cc=60; #25mph

T_cc = feedback(C_cc*P_cc,1); #PI transfer function * Cruise Control transfer function with unity feedback
t = 0:0.1:20; #runtime
step(sp_cc*T_cc,t) #Step function
