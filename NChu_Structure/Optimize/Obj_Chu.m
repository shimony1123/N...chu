function obj = testObj(x,FILE,vspvar,x0)

vspmach=vspvar(1);
vspalpha=vspvar(2);
vspbeta=vspvar(3);

des_rewrite(x,FILE);

vspaerotest(FILE);

DATA = vspaero(strcat(FILE,"_DegenGeom"),'.',0,"Mach",vspmach,"AoA",vspalpha);

CL=DATA.AERO(7)
CDtot=DATA.AERO(10)

obj = CDtot;

%{
if CL>0 & CDtot>0
    obj = -CL^(2/3)/CDtot;
else
    obj=10000;
end
%}

disp("obj:")
disp(obj)
disp("x:")
disp(x)

end