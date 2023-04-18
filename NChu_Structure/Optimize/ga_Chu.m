clear;

FILE="NChu";

x0 = [
    3,%Sec0 Twist
    1,%Sec1 Twist
    -1%Sec2 Twist
    0.12,%Sec0 T/c
    0.02,%Sec0 Camber
    0.4,%Sec0 Camber Location
    0.12,%Sec1 T/c
    0.02,%Sec1 Camber
    0.4,%Sec1 Camber Location
    0.12,%Sec2 T/c
    0.02,%Sec2 Camber
    0.4,%Sec2 Camber Location
    ];
vspvar = [0.001,0,0]; %Mach AoA Beta
A = [
    -1,1,0,0,0,0,0,0,0,0,0,0;
    0,-1,1,0,0,0,0,0,0,0,0,0;
    0,0,0,-1,0,0,1,0,0,0,0,0;
    0,0,0,0,0,0,-1,0,0,1,0,0;
    ];
b = [0,0,0,0];
Aeq = [];
beq = [];
lb = [0,-5,-8,0.06,0.01,0.25,0.06,0.01,0.25,0.06,0.01,0.25];%下限
ub = [6 ,3 ,3,0.20,0.04 ,1.0,0.20,0.04 ,1.0,0.20,0.04 ,1.0];%上限

opts = optimoptions('ga','MaxGenerations',10,'PopulationSize',10,'PlotFcn','gaplotbestf');
%,'PlotFcn','gaplotbestf'

[xopt,fval] = ga(@(x)Obj_Chu(x,FILE,vspvar,x0),12,A,b,Aeq,beq,lb,ub,[],[],opts);
disp([xopt,fval]);

x=xopt;
des_rewrite(xopt,FILE);
vspaerotest(FILE);


TR = stlread('NChu.stl');

model.Vertices = TR.Points;
model.Faces    = TR.ConnectivityList;

figure(1), clf
patch(model, 'FaceColor', [0.8 0.8 1.0])
view([20 20])
set(gca, 'DataAspectRatio',[1 1 1])