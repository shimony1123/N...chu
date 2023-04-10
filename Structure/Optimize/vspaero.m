function DATA = vspaero(casename,vspdir,stabFlag,varargin)
    nowdir = pwd;
    variables = varargin;
    if not(mod(numel(varargin),2) == 0)
        error('lack of Input');
    end
    cd(vspdir)
    delete(strcat(casename,".polar"));
    delete(strcat(casename,".history"));
    delete(strcat(casename,".stab"));
    fid = fopen(strcat(casename,".vspaero"),'r');
    i = 1;
    matchStrIndex = {};
    matchValIndex = {};
    while(feof(fid)==0)
        tline = fgetl(fid);
        TF = contains(tline,'=');
        if TF == 1
            tline = erase(tline,' ');
            extstr = rmmissing(split(tline,{'=',' '}));
            if isempty(str2num(extstr{2}))
                orgSetting{i,1} = extstr{1};
                orgSetting{i,2} = extstr{2};
            else
                orgSetting{i,1} = extstr{1};
                orgSetting{i,2} = str2num(extstr{2});
            end
            for j = 1:numel(varargin)/2
                if strcmp(orgSetting{i,1},varargin{2*j-1})
                    matchStrIndex{end+1} = i;
                    matchValIndex{end+1} = varargin{2*j};
                end
            end
        end    
        i = i+1;
    end
    fclose all;
    
    modSetting = orgSetting;
    for i = 1:numel(matchStrIndex)
        modSetting{matchStrIndex{i},2} = matchValIndex{i};
    end
    
    fid = fopen(strcat(casename,".vspaero"),'w');
    for i = 1:size(modSetting,1)
        if ischar(modSetting{i,2})
            writestr = strcat(modSetting{i,1}," = ",modSetting{i,2});
            fprintf(fid,strcat(writestr,"\n"));
        else
            writestr = strcat(modSetting{i,1}," = ",strjoin(cellstr(num2str(modSetting{i,2}')),','));
            fprintf(fid,strcat(writestr,"\n"));
        end
    end
    fclose all;
    
    if stabFlag == 1
        Command3 = strcat("./vspaero -omp 4 -stab ",casename);
    else
        Command3 = strcat("./vspaero -omp 4 ",casename);
    end
    system(Command3);
    
    DATA = VSPAERO_Results2Mat(casename);
    DATA.SETTING = modSetting;
    %DATA.AERO = rmmissing(readmatrix(strcat(casename,".polar"),'FileType','text'));
    %EXE = "vspaeroProcess3.exe";
    %command = strcat(EXE," ",casename);
    %system(command);
    %DATA = load("VSPAERO_AERO.mat");
    
    modSetting = orgSetting;
    fid = fopen(strcat(casename,".vspaero"),'w');
    for i = 1:size(modSetting,1)
        if ischar(modSetting{i,2})
            writestr = strcat(modSetting{i,1}," = ",modSetting{i,2});
            fprintf(fid,strcat(writestr,"\n"));
        else
            writestr = strcat(modSetting{i,1}," = ",strjoin(cellstr(num2str(modSetting{i,2}')),','));
            fprintf(fid,strcat(writestr,"\n"));
        end
    end
    fclose all;
    cd(nowdir);
    disp(variables)
    DATA.VAR=variables;
end



function [DATA] = VSPAERO_Results2Mat(FILENAME)
% VSPAEROの解析結果のテキストファイルをMATLAB変数に取り込むスクリプト
% 入力 FILENAME 拡張子抜きの解析結果名
% 出力 DATA 構造体 
% 2021/09/09 Ju-Hoe Kim, University of Tokyo, Tsuchiya-Ito Lab
% 抜き出すデータは現状では最低限なので別途必要な情報が有ればキムに連絡
% DATA は構造体
% DATA.AERO = Results of Aerodynamic Analysis
% %%# 抜き出したデータの並びは以下参照
%     1   2    3      4          5         6    7   8   9   10   11  12       13  14 15 16   17  18  19  20  21  22        23    
% # Mach AoA Beta Roll_rate Pitch_rate Yaw_rate CL CDo CDi CDtot CDt CDtot_t  CS L/D E CFx  CFy CFz CMx CMy CMz CDtrefftz T/QS'  
% COEFF は安定微係数
% DATA.COEFF = Results of Stability Analysis
% 安定微係数行列の集合 12行×8列が　各巡航店での安定微係数行列
% 各列の意味　1列目 微分なしの係数多分 元ファイルではTotalと表記  2　迎角微分 3 横滑り角微分 
% 4 ロール角速度微分 5 ピッチ角速度微分 6 ヨー角速度微分　7 マッハ数微分 8 速度微分　機軸方向?
% STAB_Point は安定性解析を行った平衡点
% DATA.STAB_Point = Equilibrium Point for Stability Analysis
% データの並びは次
% Mach AoA Beta Roll_rate Pitch_rate Yaw_rate
FILENAME_H = strcat(FILENAME+".history"); %% ヒストリーファイル　普通の空力解析結果が入っている。必ず存在するはず。
FILE_H_ID = fopen(FILENAME_H,"r");
if(FILE_H_ID == -1)
     % .history が開けないとき
     disp("histroy file doesn't exist.")
else
     % .history が開けたとき 
     % 実験結果の総数を取得する。
      SPLIT_REF1 = '***************************************************************************************************************************************************************************************** ';
      %SPLIT_REF2 = '  Iter      Mach       AoA      Beta       CL         CDo       CDi      CDtot      CS        L/D        E        CFx       CFy       CFz       CMx       CMy       CMz   CDtrefftz     T/QS ';
      %SPLIT_REF2 = '  Iter         Mach           AoA          Beta            CL           CDo           CDi         CDtot            CS           L/D             E           CFx           CFy           CFz           CMx           CMy           CMz       CDtrefftz      T/QS ';
      SPLIT_REF2 = '  Iter      Mach       AoA      Beta       CL         CDo       CDi      CDtot     CDt     CDtot_t      CS        L/D        E        CFx       CFy       CFz       CMx       CMy       CMz      T/QS ';
      
      BREAK_POINT = 'Skin Friction Drag Break Out:';
      Buff_count1 = []; % 行数カウント 単位がバイトなのに注意 Pythonみたいに行番号にしてほしい
      Buff_count2 = []; % 行数カウント 単位がバイトなのに注意 Pythonみたいに行番号にしてほしい
      tline = 'Empty'; %% char なのでシングルクォーテーションで囲む
      while(ischar(tline)==1) 
          
          tline = fgetl(FILE_H_ID);
          %disp(tline)
          %pause()
          if(strcmp(tline,SPLIT_REF1))
              Buff_count1 = [Buff_count1, ftell(FILE_H_ID)];
          end
          if(strcmp(tline,SPLIT_REF2))
              Buff_count2 = [Buff_count2, ftell(FILE_H_ID)];
          end
          
          
      end
      
      Nsim = length(Buff_count1); %% シミュレーション総数
      AERO = zeros(Nsim,22); %空力データ配列の確保 AoA から T/QS に安定性解析時に変化する 三方向の角速度を足して　列数は21
      AngularV = zeros(1,3); %角速度格納用 
      
      %WakeIteration 回数を計測する。　.vspaeroから直接読むか明示的に与えてもよいが.historyだけで完結するようにした。
     NWake = 0; %% 初期化
     fseek(FILE_H_ID,Buff_count2(1),-1);
     tline = fgetl(FILE_H_ID);
     
     Wake_Check = '';
     while(strcmp(tline,Wake_Check)==0)
         NWake = NWake+1;
         tline = fgetl(FILE_H_ID);
         %disp(tline)
        % 中途半端に打ち切ると無限ループに入ってしまう。 
         if( (tline==-1))
             break
         end
         
     end
      
      
      for i=1:Nsim
          fseek(FILE_H_ID,Buff_count1(i),-1);
          tline = fgetl(FILE_H_ID);
          % シミュレーション結果のブロックごとに処理
          while(strcmp(tline,BREAK_POINT)==0)
              % 中途半端に打ち切ると無限ループに入ってしまう。 
             if( (tline==-1))
                 break
             end
             c = strsplit(string(tline)); %% string(char)でcharをStringに変換
           % 最初にRoll,Pitch,Yaw の角速度を抜き出す
             if(strcmp(c(1),"Roll__Rate")==1)
                AngularV(1,1) = str2double(c(2)); %Roll
                tline = fgetl(FILE_H_ID); %Pitch
                c = strsplit(string(tline)); %Pitch
                AngularV(1,2) = str2double(c(2)); % Pitch
                tline = fgetl(FILE_H_ID); %Yaw
                c = strsplit(string(tline)); %% string(char)でcharをStringに変換 %Yaw
                AngularV(1,3) = str2double(c(2)); % Yaw
             end
           % 角速度を抜き出したら他の空力データを抜き出す
             if(strcmp(tline,SPLIT_REF2)==1)
                for j=1:NWake
                    tline = fgetl(FILE_H_ID);
                end
                c = (strsplit(string(tline))); %% string(char)でcharをStringに変換
                c = (c(3:end));
                Param = zeros(1,length(c));
                for k=1:length(c)
                   Param(k) = str2double(c(k)); 
                end
                AERO(i,:) = [Param(1:3), AngularV, Param(4:end)];
             end 
             tline = fgetl(FILE_H_ID);
          end
      end
      
% 空力データの抜き出し終わり
DATA.AERO = AERO;     
fclose(FILE_H_ID);
end

% ここから.stabを抜き出す
FILENAME_S = strcat(FILENAME+".stab"); %% stab　微係数が入っている
FILE_S_ID = fopen(FILENAME_S,"r");
if(FILE_S_ID == -1)
     % .stab が開けないとき
     % 何もせずスルー
else
     % .stab が開けたとき
      % 実験結果の総数を取得する。
      SPLIT_REF1 = '***************************************************************************************************************************************************************************************** ';
      SPLIT_REF2 = '#             Base    Derivative:                                                                                                         ';
      BREAK_POINT ='# Result                 Value      Units             ';
      Buff_count1 = []; % 行数カウント 単位がバイトなのに注意 Pythonみたいに行番号にしてほしい
      Buff_count2 = []; % 行数カウント 単位がバイトなのに注意 Pythonみたいに行番号にしてほしい
      tline = 'Empty'; %% char なのでシングルクォーテーションで囲む
      while(ischar(tline)==1) 
          
          tline = fgetl(FILE_S_ID);
          %disp(tline)
          %pause()
          if(strcmp(tline,SPLIT_REF1))
              Buff_count1 = [Buff_count1, ftell(FILE_S_ID)];
          end
          if(strcmp(tline,SPLIT_REF2))
              Buff_count2 = [Buff_count2, ftell(FILE_S_ID)];
          end
          
          
      end
      
      Nsim = length(Buff_count1); %% シミュレーション総数
      % 舵面の個数に応じて動的にファイル形式が変化するので 微分した変数の個数を探す
      % SPLIT_REF2から2行下の文字数個数をカウントする
      fseek(FILE_S_ID,Buff_count2(1),-1);
      fgetl(FILE_S_ID); % 1行下へ
      tline = fgetl(FILE_S_ID); % 1行下へ
      COEFF_NAME = strsplit(string(tline));
      Ndiv = length(COEFF_NAME)-3;
      COEFF = zeros(12,Ndiv,Nsim);
      STAB_Point = zeros(Nsim,8);
      
       for i=1:Nsim
          fseek(FILE_S_ID,Buff_count1(i),-1);
          tline = fgetl(FILE_S_ID);
          % シミュレーション結果のブロックごとに処理
          while(strcmp(tline,BREAK_POINT)==0)
             c = strsplit(string(tline)); %% string(char)でcharをStringに変換
           % 最初に平衡点の情報を抜き出す
             if(strcmp(c(1),"Mach_")==1)
                STAB_Point(i,1) = str2double(c(2)); %Mach
                
                tline = fgetl(FILE_S_ID); %AoA
                c = strsplit(string(tline)); %AoA
                STAB_Point(i,2) = str2double(c(2)); % AoA
                
                tline = fgetl(FILE_S_ID); %Beta
                c = strsplit(string(tline)); %Beta
                STAB_Point(i,3) = str2double(c(2)); %Beta
                
                tline = fgetl(FILE_S_ID); %Rho
                c = strsplit(string(tline)); %Rho
                STAB_Point(i,4) = str2double(c(2)); %Rho
                
                tline = fgetl(FILE_S_ID); %Vinf
                c = strsplit(string(tline)); %Vinf
                STAB_Point(i,5) = str2double(c(2)); %Vinf
                
                tline = fgetl(FILE_S_ID); %Roll_rate
                c = strsplit(string(tline)); %Roll_rate
                STAB_Point(i,6) = str2double(c(2)); %Roll_rate     
                
                tline = fgetl(FILE_S_ID); %Pitch_rate
                c = strsplit(string(tline)); %Pitch_rate
                STAB_Point(i,8) = str2double(c(2)); %Pitch_rate     
                
                tline = fgetl(FILE_S_ID); %Yaw_rate
                c = strsplit(string(tline)); %Yaw_rate
                STAB_Point(i,8) = str2double(c(2)); %Yaw_rate     
                
             end
           % 平衡点を抜き出した後は微係数を求める。
             if(strcmp(tline,SPLIT_REF2)==1)
                for j=1:5 %% 目的の情報まで行を飛ばす
                    tline = fgetl(FILE_S_ID);
                end
                
                for j=1:12
                   tline = fgetl(FILE_S_ID);
                   c = (strsplit(string(tline))); %% string(char)でcharをStringに変換
                   c = (c(3:end-1));
                   Param = zeros(1,Ndiv);
                   for k=1:Ndiv
                      Param(k) = str2double(c(k)); 
                   end
                   COEFF(j,:,i) = Param;
                end
             end 
             tline = fgetl(FILE_S_ID);
          end
      end
      
DATA.COEFF_NAME = COEFF_NAME(3:end-1);
DATA.COEFF = COEFF;
DATA.STAB_Point = STAB_Point;
fclose(FILE_S_ID);

end
end
