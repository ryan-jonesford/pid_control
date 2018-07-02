#clear out past variables
clc
# load the control package
pkg load control

# Car cruise control model
m=1800; #Mass of car in Kg
b=50; #Viscous friction coefficient


# PID transfer function setup with initial guess for cruise control
Kp_cc=50;
Kd_cc=500;
Ki_cc=2.75;

s_cc=tf('s');
C_cc=pid(Kp_cc,Ki_cc,Kd_cc);

P_cc = 1/(s_cc*(m*s_cc + b)); #Cruise Control Transfer function

# Setpoints
sp_cc=10; #mph

T_cc = feedback(C_cc*P_cc,1); #PI transfer function * Cruise Control transfer function with unity feedback
t = 0:0.1:100; #runtime
#step(sp_cc*T_cc,t) #Step function

# PID transfer function setup with initial guess for steering
Kp_s=.5;
Kd_s=.04;
Ki_s=0.0005;

s_s=tf('s');
C_s=pid(Kp_s,Ki_s,Kd_s);

P_s=(1/(s_s^2+1));
sp_s=.2;

T_s=feedback(C_s,P_s,1);

step(sp_s*T_s,t)

