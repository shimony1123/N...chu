clear;

x0 = [0.2,0.2,0.2,1.0,1.0];
vspvar = [0,0]; %Mach AoA Beta
A = 1;
b = 1;
a = 1;
options = optimset('PlotFcns','optimplotfval');

[xopt,fval] = fminsearch(@(x)testObj(x,vspvar),x0,options);
disp([xopt,fval]);