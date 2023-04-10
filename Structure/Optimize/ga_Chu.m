clear;



x0 = [4.5,-1,-2,0.12,0.02,0.4,0.12,0.02,0.4,0.12,0.02,0.4];
vspvar = [0.001,0,0]; %Mach AoA Beta
A = [-1,1,0,0,0,0,0,0,0,0,0,0;0,-1,1,0,0,0,0,0,0,0,0,0;0,0,0,-1,0,0,1,0,0,0,0,0;0,0,0,0,0,0,-1,0,0,1,0,0];
b = [0,0,0,0];
Aeq = [];
beq = [];
lb = [3,-5,-8,0.06,0.01,0.0,0.06,0.01,0.0,0.06,0.01,0.0];%下限
ub = [6,3,3,0.36,0.06,1.0,0.36,0.06,1.0,0.36,0.06,1.0];%上限

options = optimoptions('ga','MaxGenerations',10,'PopulationSize',50,'PlotFcn','gaplotbestf');

[xopt,fval] = ga(@(x)Obj_Chu(x,vspvar,x0),12,A,b,Aeq,beq,lb,ub,[],options);
disp([xopt,fval]);

x=xopt;
test_des_rewrite;
vspaero_Chu;


TR = stlread('NChu.stl');

model.Vertices = TR.Points;
model.Faces    = TR.ConnectivityList;

figure(1), clf
patch(model, 'FaceColor', [0.8 0.8 1.0])
view([20 20])
set(gca, 'DataAspectRatio',[1 1 1])