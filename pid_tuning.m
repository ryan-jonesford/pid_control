#clear out past variables
clc
# load the control package
pkg load control

# Car cruise control model
m=1000; #Mass of car in Kg
b=.1; #Viscous friction coefficient


# PID transfer function setup with initial guess for cruise control
Kp_cc=1000;
Ki_cc=.001;
Kd_cc=50;

s_cc=tf('s');
C_cc=pid(Kp_cc,Ki_cc,Kd_cc);

P_cc = 1/(m*s_cc + b); #Cruise Control Transfer function

# Setpoints
sp_cc=55; #mph
sp_s=.5; #-1 to 1 for steering 

T_cc = feedback(C_cc*P_cc,1); #PI transfer function * Cruise Control transfer function with unity feedback
t = 0:0.1:100; #runtime
step(sp_cc*T_cc,t) #Step function