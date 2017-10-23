
% 组合导航新协议rev3.9  帧3 100字节
% 帧头 AA 55 03
% 注意：输入文本文件需要把帧对齐，删掉多余数据
% 技术支持：QQ467080191
close all;clear all;clc;
FileName='3.txt';
fid = fopen(FileName, 'r' );
data0 = fread(fid,inf,'uchar');
fclose(fid);
len0 = length(data0);
len=100;
data = reshape(data0,len,len0/len);
clear data0;
n=len0/len;
ts=0.01;  

Gyro=zeros(n,3);
Acc=zeros(n,3);
Magn=zeros(n,4);
HBar=zeros(n,2);
INS_Att=zeros(n,3);
INS_Vel=zeros(n,3);
INS_Pos=zeros(n,3);
Scenes=zeros(n,1);
Mode=zeros(n,1);
temperature=zeros(n,1);
Cnt=zeros(n,1);


for i=1:1:n 
    %Gyro 
    G1 = 65536*data(5,i)+256*data(6,i)+data(7,i);    % Gyro_x
    G2 = 65536*data(8,i)+256*data(9,i)+data(10,i);    % Gyro_y
    G3 = 65536*data(11,i)+256*data(12,i)+data(13,i);  % Gyro_z
    if G1>=2^23
        G1 = G1 - 2^24;
    end
    if G2>=2^23
        G2 = G2 - 2^24;
    end
    if G3>=2^23
        G3 = G3 - 2^24;
    end
    Gyro(i,:) = [G1,G2,G3]*1e-4; % Gyro_x/y/z 单位：°/s  
    
   %加计输出  
    A1 = 65536*data(14,i)+256*data(15,i)+data(16,i);    % Acc_x
    A2 = 65536*data(17,i)+256*data(18,i)+data(19,i);    % Acc_y
    A3 = 65536*data(20,i)+256*data(21,i)+data(22,i);    % Acc_z
    if A1>=2^23
        A1 = A1 - 2^24;
    end
    if A2>=2^23
        A2 = A2 - 2^24;
    end
    if A3>=2^23
        A3 = A3 - 2^24;
    end    
    Acc(i,:) = [A1,A2,A3]*1e-5;  % Acc_x/y/z 单位：g 
     
   %磁输出  
    M1 = 256*data(23,i)+data(24,i);    % Magn_x
    M2 = 256*data(25,i)+data(26,i);    % Magn_y
    M3 = 256*data(27,i)+data(28,i);    % Magn_z
    if M1>=2^15
        M1 = M1 - 2^16;
    end
    if M2>=2^15
        M2 = M2 - 2^16;
    end
    if M3>=2^15
        M3 = M3 - 2^16;
    end
    Magn(i,1:1:3) = [M1,M2,M3]*1e-2; % 东北天  Magn_x/y/z 单位：uT
    
    %气压计输出  
    H = 65536*data(29,i)+256*data(30,i)+data(31,i);     
    if H>2^23
        H=H-2^24;
    end  
    HBar(i,1) = H*0.01; % 海拔 单位：米
    
    % flag   0--无效 1--有效
    temp1=data(32,i);
    Magn(i,4) = bitand(temp1,1); 
    HBar(i,2) = bitshift(bitand(temp1,2),-1);% bitshift 为正表示左移，为负表示右移

    % INS_Att  
    pitch=256*data(66,i)+data(67,i);
    roll=256*data(68,i)+data(69,i);
    yaw=256*data(70,i)+data(71,i);
    if pitch>=2^15
        pitch = pitch - 2^16;
    end
    if roll>=2^15
        roll = roll - 2^16;
    end
    INS_Att(i,:) = [pitch,roll,yaw]*1e-2;% 单位：°
     
    %INS.Vn  
    VE1=65536*data(72,i)+256*data(73,i)+data(74,i); 
    VN1=65536*data(75,i)+256*data(76,i)+data(77,i); 
    VU1=65536*data(78,i)+256*data(79,i)+data(80,i); 
    if VE1>=2^23
        VE1 = VE1- 2^24;
    end
    if VN1>=2^23
        VN1 = VN1 - 2^24;
    end
    if VU1>=2^23
        VU1 =VU1 - 2^24;
    end
    INS_Vel(i,:) = [VE1,VN1,VU1]*1e-4;% 单位：m/s
    
    %INS.Pos  
    Lon1=16777216*data(81,i)+65536*data(82,i)+256*data(83,i)+data(84,i); %单位：° 经度
    Lat1=16777216*data(85,i)+65536*data(86,i)+256*data(87,i)+data(88,i); %单位：° 纬度
    hMSL1=65536*data(89,i)+256*data(90,i)+data(91,i); %单位：m  海拔
    if Lon1>=2^31
        Lon1 = Lon1- 2^32;
    end
    if Lat1>=2^31
        Lat1 = Lat1 - 2^32;
    end
    if hMSL1>=2^23
        hMSL1 =hMSL1 - 2^24;
    end
    INS_Pos(i,:) = [Lon1*1e-7,Lat1*1e-7,hMSL1*1e-2]; % 经度 纬度 海拔
     
    % 场景与模式
    temp3=data(92,i);
    Mode(i,1) = bitand(temp3,15);      % 工作模式  ALIGN = 1    INS = 2   AHRS= 3     VG= 4  
    Scenes(i,1) = bitshift(bitand(temp3,240),-4);   % 工作场景 1--车载 2--室内 3--船载 4--固定翼 5--旋翼
    
    % 温度
    temper= 256*data(97,i)+data(98,i);
    if temper>=2^15
        temper =temper - 2^16;
    end  
    temperature(i,:)=temper*0.01;
    
    % 计数
    temp = data(99,i);  
    Cnt(i,:) = temp; % 单位：ms  
    
end

t=[1:1:n]*ts;

figure('name','Gyro'); 
subplot(3,1,1);plot(t,Gyro(:,1));grid on; ylabel('Gyro X /°/s');xlabel('t/s');
subplot(3,1,2);plot(t,Gyro(:,2));grid on; ylabel('Gyro Y /°/s');xlabel('t/s');
subplot(3,1,3);plot(t,Gyro(:,3));grid on; ylabel('Gyro Z /°/s');xlabel('t/s');

figure('name','Acc'); 
subplot(3,1,1);plot(t,Acc(:,1));grid on; ylabel('Acc X /g');xlabel('t/s');
subplot(3,1,2);plot(t,Acc(:,2));grid on; ylabel('Acc Y /g');xlabel('t/s');
subplot(3,1,3);plot(t,Acc(:,3));grid on; ylabel('Acc Z /g');xlabel('t/s');

figure('name','Magn'); 
subplot(4,1,1);plot(t,Magn(:,1));grid on; ylabel('Magn X /uT');xlabel('t/s');
subplot(4,1,2);plot(t,Magn(:,2));grid on; ylabel('Magn Y /uT');xlabel('t/s');
subplot(4,1,3);plot(t,Magn(:,3));grid on; ylabel('Magn Z /uT');xlabel('t/s');
subplot(4,1,4);plot(t,Magn(:,4));grid on; xlabel('t/s');

figure('name','HBar'); 
subplot(2,1,1);plot(t,HBar(:,1));grid on; ylabel('HBar /m');xlabel('t/s');
subplot(2,1,2);plot(t,HBar(:,2));grid on; xlabel('t/s');

figure('name','工作模式'); 
subplot(2,1,1);plot(t,Mode(:,1),'r');grid on; hold on; plot(t,Scenes(:,1),'b');legend('模式','场景');xlabel('t/s');   ylim([-0.2,5.2]); % Mode: ALIGN=1 INS=2 AHRS=3 VG=4   Scenes: 1-车载 2-室内 3-船载 4-固定翼 5-旋翼
subplot(2,1,2);plot(t,temperature);grid on; xlabel('t/s'); ylabel('/℃');legend('温度');

figure('name','姿态');
subplot(3,1,1);plot(t,INS_Att(:,1));grid on; ylabel('Pitch /°');xlabel('t/s');
subplot(3,1,2);plot(t,INS_Att(:,2));grid on; ylabel('Roll /°');xlabel('t/s');
subplot(3,1,3);plot(t,INS_Att(:,3));grid on; ylabel('Yaw /°'); xlabel('t/s');

figure('name','速度');
subplot(3,1,1);plot(t,INS_Vel(:,1),'r');grid on;  ylabel('东向速度 /m/s');xlabel('t/s');
subplot(3,1,2);plot(t,INS_Vel(:,2),'r');grid on;  ylabel('北向速度 /m/s');xlabel('t/s');
subplot(3,1,3);plot(t,INS_Vel(:,3),'r');grid on;  ylabel('天向速度 /m/s');xlabel('t/s');

figure('name','位置');
subplot(3,1,1);plot(t,INS_Pos(:,1),'r');grid on; ylabel('经度lon /度'); xlabel('t/s');
subplot(3,1,2);plot(t,INS_Pos(:,2),'r');grid on; ylabel('纬度lat /度'); xlabel('t/s');
subplot(3,1,3);plot(t,INS_Pos(:,3),'r');grid on; ylabel('海拔hMSL /m'); xlabel('t/s');

