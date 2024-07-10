%%
clc
clear
syms tetha1 tetha2 l1 l2 l3 tetha3
T1 = [cos(tetha1) -sin(tetha1) 0 0;
    sin(tetha1) cos(tetha1) 0 0;
    0 0 1 0;
    0 0 0 1];
T2 = [cos(tetha2) -sin(tetha2) 0 l1;
    sin(tetha2) cos(tetha2) 0 0;
    0 0 1 0;
    0 0 0 1];
T3 = [cos(tetha3) -sin(tetha3) 0 l2;
    sin(tetha2) cos(tetha2) 0 0;
    0 0 1 0;
    0 0 0 1];
T4 = [1 0 0 l3;
    0 1 0 0;
    0 0 1 0;
    0 0 0 1];
Tf = simplify(T1*T2*T3)
%%
Tf = double(subs(Tf,[tetha1,tetha2,tetha3,l1,l2],[deg2rad(10),deg2rad(15),deg2rad(19.89),0.5,0.4])*[0.3;0;0;1])
%%
R1 = [cosd(10) -sind(10) 0;
    sind(10) cosd(10) 0;
    0 0 1];
R2 = [cosd(15) -sind(15) 0;
    sind(15) cosd(15) 0;
    0 0 1];
R3 = R1*R2
%%
H = [cosd(45) -sind(45) 0;
    sind(45) cosd(45) 0;
    0 0 1];
L = R3^(-1)*H