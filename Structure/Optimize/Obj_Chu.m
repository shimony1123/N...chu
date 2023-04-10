function obj = testObj(x,vspvar,x0)

disp(x);

test_des_rewrite;

vspaero_Chu;

CL=DATA.AERO(7)
CDtot=DATA.AERO(10)

%obj = CDtot;
%+10000*(CL-0.5238)^2;
obj = -CL/CDtot;


disp("obj:")
disp(obj)
disp("x:")
disp(x)

end