% Initialization:
clc;clear;close all;
% Initial Data:
g = 9.81;jx=7.5e-3;jy=jx;jz=1.3e-2;m=0.65;
% Creating state space matrix A B C D:
A = zeros(12);
A(1,4)=1;
A(2,5)=1;
A(3,6)=1;
A(7,2)=-9.81;
A(8,1)=9.81;
A(10,7)=1;
A(11,8)=1;
A(12,9)=1;
B = zeros(12,4);
B(4,2)=1/jx;
B(5,3)=1/jx;
B(6,4)=1/jz;
B(9,1)=1/m;
C=zeros(4,12);
C(1,1)=1;C(2,2)=1;C(3,3)=1;C(4,12)=1;%C(4,10)=1;C(5,11)=1;
D = zeros(4,4);
s = tf('s');
% Obtaining primary Transfer Function
G_s = ss(A,B,C,D);
G_total = tf(G_s);
G_y4_u1 = G_total(4,1);
G_y3_u4 = G_total(3,4);
G_y2_u3 = G_total(2,3);
G_y1_u2 = G_total(1,2);

%%
%Design via PID Tuner: 

% Designing PD controller for the G_y4_u1 = y4/u1:
% our desired values: P.O < 20% and Ts < 1s;
% By use of the PID Tuner app in matlab:
Kp1 = 17.429;
Ki1 = 0;
Kd1 = 5.2958;
PD_y4 = pid(Kp1,Ki1,Kd1);
G_y4_u1_compensated = G_y4_u1 * PD_y4;
T_y4_u1_cl = feedback(G_y4_u1_compensated,1);
figure(1);hold on;
subplot(2,2,1);
step(T_y4_u1_cl);grid on;
stepinfo(T_y4_u1_cl);
subplot(2,2,2);
bode(G_y4_u1_compensated);grid on;
subplot(2,2,3);
nyqlog(G_y4_u1_compensated);grid on;
subplot(2,2,4);
rlocus(G_y4_u1_compensated);grid on;
hold off;


% Designing PD controller for the G_y3_u4 = y3/u4:
% our desired values: P.O < 20% and Ts < 1s;
% By use of the PID Tuner app in matlab:
Kp2 = 0.404;
Ki2 = 0;
Kd2 = 0.11403;
PD_y3 = pid(Kp2,Ki2,Kd2);
G_y3_u4_compensated = G_y3_u4 * PD_y3;
T_y3_u4_cl = feedback(G_y3_u4_compensated,1);
figure(2);hold on;
subplot(2,2,1);
step(T_y3_u4_cl);grid on;
stepinfo(T_y3_u4_cl);
subplot(2,2,2);
bode(G_y3_u4_compensated);grid on;
subplot(2,2,3);
nyqlog(G_y3_u4_compensated);grid on;
subplot(2,2,4);
rlocus(G_y3_u4_compensated);grid on;
hold off;


% Designing PD controller for the G_y2_u3 = y2/u3:
% our desired values: P.O < 20% and Ts < 1s;
% By use of the PID Tuner app in matlab:
Kp3 = 0.22051;
Ki3 = 0;
Kd3 = 0.063985;
PD_y2 = pid(Kp3,Ki3,Kd3);
G_y2_u3_compensated = G_y2_u3 * PD_y2;
T_y2_u3_cl = feedback(G_y2_u3_compensated,1);
figure(3);hold on;
subplot(2,2,1);
step(T_y2_u3_cl);grid on;
stepinfo(T_y2_u3_cl);
subplot(2,2,2);
bode(G_y2_u3_compensated);grid on;
subplot(2,2,3);
nyqlog(G_y2_u3_compensated);grid on;
subplot(2,2,4);
rlocus(G_y2_u3_compensated);grid on;
hold off;


% Designing PD controller for the G_y1_u2 = y1/u2:
% our desired values: P.O < 20% and Ts < 1s;
% By use of the PID Tuner app in matlab:
Kp4 = 0.22051;
Ki4 = 0;
Kd4 = 0.063985;
PD_y1 = pid(Kp4,Ki4,Kd4);
G_y1_u2_compensated = G_y1_u2 * PD_y1;
T_y1_u2_cl = feedback(G_y1_u2_compensated,1);
figure(4);hold on;
subplot(2,2,1);
step(T_y1_u2_cl);grid on;
stepinfo(T_y1_u2_cl);
subplot(2,2,2);
bode(G_y1_u2_compensated);grid on;
subplot(2,2,3);
nyqlog(G_y1_u2_compensated);grid on;
subplot(2,2,4);
rlocus(G_y1_u2_compensated);grid on;
hold off;

%%
% Design by calculations:

% Designing PD controller:
% our desired values: P.O < 20% and Ts < 1s;
% Because all of our denominators are same, the location of the PD zero is
% equal for all of them:
Z_loc = 2.327; %Calculated with the phase condition
% Calculated gains for each design:
K_cal = [5.5342 0.1107 0.0639 0.0639];

% Outputs with calculated data:

% G_y4_u1 = y4/u1
G_y4_u1_comp_cal = K_cal(1)*(s+2.327)*G_y4_u1;
T_y4_u1_cal_cl = feedback(G_y4_u1_comp_cal,1);
figure(1);hold on;
subplot(2,2,1);
step(T_y4_u1_cal_cl);grid on;
stepinfo(T_y4_u1_cal_cl);
subplot(2,2,2);
bode(G_y4_u1_comp_cal);grid on;
margin(G_y4_u1_comp_cal);
subplot(2,2,3);
nyqlog(G_y4_u1_comp_cal);grid on;
subplot(2,2,4);
rlocus(G_y4_u1_comp_cal);grid on;
hold off;

% G_y3_u4 = y3/u4
G_y3_u4_comp_cal = K_cal(2)*(s+2.327)*G_y3_u4;
T_y3_u4_cal_cl = feedback(G_y3_u4_comp_cal,1);
figure(2);hold on;
subplot(2,2,1);
step(T_y3_u4_cal_cl);grid on;
stepinfo(T_y3_u4_cal_cl);
subplot(2,2,2);
bode(G_y3_u4_comp_cal);grid on;
margin(G_y3_u4_comp_cal);
subplot(2,2,3);
nyqlog(G_y3_u4_comp_cal);grid on;
subplot(2,2,4);
rlocus(G_y3_u4_comp_cal);grid on;
hold off;

% G_y2_u3 = y2/u3
G_y2_u3_comp_cal = K_cal(3)*(s+2.327)*G_y2_u3;
T_y2_u3_cal_cl = feedback(G_y2_u3_comp_cal,1);
figure(3);hold on;
subplot(2,2,1);
step(T_y2_u3_cal_cl);grid on;
stepinfo(T_y2_u3_cal_cl);
subplot(2,2,2);
bode(G_y2_u3_comp_cal);grid on;
margin(G_y2_u3_comp_cal);
subplot(2,2,3);
nyqlog(G_y2_u3_comp_cal);grid on;
subplot(2,2,4);
rlocus(G_y2_u3_comp_cal);grid on;
hold off;

% G_y1_u2 = y1/u2
G_y1_u2_comp_cal = K_cal(4)*(s+2.327)*G_y1_u2;
T_y1_u2_cal_cl = feedback(G_y1_u2_comp_cal,1);
figure(4);hold on;
subplot(2,2,1);
step(T_y1_u2_cal_cl);grid on;
stepinfo(T_y1_u2_cal_cl);
subplot(2,2,2);
bode(G_y1_u2_comp_cal);grid on;
margin(G_y1_u2_comp_cal);
subplot(2,2,3);
nyqlog(G_y1_u2_comp_cal);grid on;
subplot(2,2,4);
rlocus(G_y1_u2_comp_cal);grid on;
hold off;

%%
% Here to get setteling time les than 1s we changed the gains by expriment:

% Changed gain to achieve better setteling time:
K_changed = [10 0.2 0.2 0.2];

% Outputs with calculated data:

% G_y4_u1 = y4/u1
G_y4_u1_comp_exp = K_changed(1)*(s+2.327)*G_y4_u1;
T_y4_u1_exp_cl = feedback(G_y4_u1_comp_exp,1);
figure(1);hold on;
subplot(2,2,1);
step(T_y4_u1_exp_cl);grid on;
stepinfo(T_y4_u1_exp_cl)
subplot(2,2,2);
bode(G_y4_u1_comp_exp);grid on;
margin(G_y4_u1_comp_exp);
subplot(2,2,3);
nyqlog(G_y4_u1_comp_exp);grid on;
subplot(2,2,4);
rlocus(G_y4_u1_comp_exp);grid on;
hold off;

% G_y3_u4 = y3/u4
G_y3_u4_comp_exp = K_changed(2)*(s+2.327)*G_y3_u4;
T_y3_u4_exp_cl = feedback(G_y3_u4_comp_exp,1);
figure(2);hold on;
subplot(2,2,1);
step(T_y3_u4_exp_cl);grid on;
stepinfo(T_y3_u4_exp_cl)
subplot(2,2,2);
bode(G_y3_u4_comp_exp);grid on;
margin(G_y3_u4_comp_exp);
subplot(2,2,3);
nyqlog(G_y3_u4_comp_exp);grid on;
subplot(2,2,4);
rlocus(G_y3_u4_comp_exp);grid on;
hold off;

% G_y2_u3 = y2/u3
G_y2_u3_comp_exp = K_changed(3)*(s+2.327)*G_y2_u3;
T_y2_u3_exp_cl = feedback(G_y2_u3_comp_exp,1);
figure(3);hold on;
subplot(2,2,1);
step(T_y2_u3_exp_cl);grid on;
stepinfo(T_y2_u3_exp_cl)
subplot(2,2,2);
bode(G_y2_u3_comp_exp);grid on;
margin(G_y2_u3_comp_exp);
subplot(2,2,3);
nyqlog(G_y2_u3_comp_exp);grid on;
subplot(2,2,4);
rlocus(G_y2_u3_comp_exp);grid on;
hold off;

% G_y1_u2 = y1/u2
G_y1_u2_comp_exp = K_changed(4)*(s+2.327)*G_y1_u2;
T_y1_u2_exp_cl = feedback(G_y1_u2_comp_exp,1);
figure(4);hold on;
subplot(2,2,1);
step(T_y1_u2_exp_cl);grid on;
stepinfo(T_y1_u2_exp_cl)
subplot(2,2,2);
bode(G_y1_u2_comp_exp);grid on;
margin(G_y1_u2_comp_exp);
subplot(2,2,3);
nyqlog(G_y1_u2_comp_exp);grid on;
subplot(2,2,4);
rlocus(G_y1_u2_comp_exp);grid on;
hold off;

%%


