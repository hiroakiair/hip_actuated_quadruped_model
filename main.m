clear; format compact   %initialize
addpath('./eom')  
addpath('./symbolic')  
addpath('./fig') 
%微分方程式を解く設定
global g
global tfinal

global M J_1 l_b % body
global m J_2 l_ab % viscera
global k_ab b_ab  % stiffness of abdominal oblique muscle
global g

global k_F k_H b_F b_H L_0F L_0H
global tau_nF tau_nH gamma_F_td gamma_H_td
global xF_toe xH_toe

%解の保存変数
global tout_ac qout_ac lout_ac gout_ac teout_ac qeout_ac ieout_ac phaseout_ac

%体幹の設定    
M = 3;
l_b = 0.25;
J_1 = M*l_b^2/3;


%腹部の設定
m = 1;
l_ab = 0.2;
J_2 = m*l_ab^2;
k_ab = 80.0;
% k_ab = 200.0;
b_ab = 10.0;

g = 9.81;

%脚の設定
k_F = 2000;
b_F = 0;
k_H = 2500;
b_H = 0;

L_0F = 0.28;
L_0H = 0.3;

xF_toe = 0;
xH_toe = 0;

tau_nF = 10;
tau_nH = 12;
gamma_F_td = pi/8;
gamma_H_td = pi/8;
%comment

%設置位置

%解の保存
tout_ac = [];
qout_ac = [];
lout_ac = [];
gout_ac = [];
teout_ac = [];
qeout_ac = [];
ieout_ac = [];
phaseout_ac = [];


%微分方程式を解く設定
tfinal = 10;

        
% モデルの選択
q0 = [0; L_0F+0.1;pi/8; pi/2+pi/8];
dq0 = [0.8; 0; 0; 0];
q_initial = [q0; dq0];

%bounding
bounding(q_initial);

%解の表示
plot_sols(tout_ac,qout_ac);
animation(tout_ac,qout_ac,lout_ac, gout_ac);