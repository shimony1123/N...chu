clear;

x0 = 100;
vspvar = [0.02,0,0]; %Mach AoA Beta
A = 1;
b = 1;
a = 1;
options = optimoptions('fmincon','FiniteDifferenceStepSize',1.0000e-01);

[xopt,fval] = fmincon(@(x)testObj(x,vspvar),x0,[],[],[],[],0,1);
disp([xopt,fval]);