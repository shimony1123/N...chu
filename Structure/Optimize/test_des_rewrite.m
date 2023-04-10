^
%val_origin = '0.2';


des = readlines('test_1_2.des');%元のdesファイルをstr型で取得している
fileID = fopen('test_2.des','w');%確認用に元のファイルを残したかったため別のファイルを作っている
val_num=des(1);

fprintf(fileID, '%1s\r\n', val_num);

for i = 2:str2double(val_num)+1;

%disp(des)

val_new = " "+string(x(i-1)); %新しく適応したい数字
strs=strsplit(des(i),':');
digits = strs(end);
val_origin = digits;
new_des2 = strrep(des(i),val_origin,val_new);%str型でval_originと一致する部分をval_newに置き換えている
%disp(new_des2)
fprintf(fileID, '%1s\r\n', new_des2);%置き換えた数値を新たなdesファイルに書き込み
%disp(i)
end
%{
digits = extract(des(2),digitsPattern) %digitsPatternで数字だけを抜き出せる
n = length(digits)
val_origin = strcat(digits(n-1),'.',digits(n)) %テキストファイル上で数字が存在する位置を無理やり指定している。桁が増える場合は書き換え必要
%}