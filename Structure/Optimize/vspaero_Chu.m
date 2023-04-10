%vspによる空力計算
%clear;

FILE = "NChu";
ScriptName = "test";
nowdir = pwd; 
vspdir = "."; %VSP.exeがあるディレクトリ

%disp(vspdir)
% VSP
CommandVSP = "start /min ./vsp NChu.vsp3 -des test_2.des -script test.vspscript";
%strcat("start /min vsp ",FILE,".vsp3"," -script ",ScriptName,".vspscript"); %手順2のコマンド環境に合わせて各自書き換え
%disp(CommandVSP)

% .vspscriptは別途用意  
% cmd から powershell に変更
[~,~] = system(CommandVSP);
pause(3);
% 待ち時間を経過しても処理が終わらなければ強制停止する
[~,Task_Cmdout] = system("taskkill /IM vsp.exe /F");
%{
vspmach = vspvar(1);
vspalpha = vspvar(2);
vspbeta =vspvar(3);
%}
vspmach = 0.02;
vspalpha = 0;
vspbeta = 0;
DATA = vspaero(strcat(FILE,"_DegenGeom"),vspdir,0,"Mach",vspmach,"AoA",vspalpha);
