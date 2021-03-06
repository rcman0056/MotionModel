%VO Velocities Comparison to Velocites from Calculated GPS Position
%close all
%clear all
Save_Name = 'oneloopmmVOR_Inertial';
Title_Super =['VO Velocity Check  ' Save_Name];
FigNum=18;
%Pull in data byte array for filter and convert

filename= ['/home/suas/IdeaProjects/MotionModel/Scorpion_Fain/Filter_Output/SampleRun_' Save_Name '.txt'];
fid = fopen(filename, 'r');
data = fread(fid,'double',0,'b');
fclose(fid);
numCols = 34; %Set num of cols in data bytearray so matlab can parse the data
data = reshape(data,numCols,[])';
 
%Pull in data byte array for pixhawk data and convert
filename= ['/home/suas/IdeaProjects/MotionModel/Scorpion_Fain/Filter_Output/Pixhawk_Data_'  Save_Name '.txt'];
fid = fopen(filename, 'r');
data_pix = fread(fid,'double',0,'b');
fclose(fid);
numCols = 6; %Set num of cols in data bytearray so matlab can parse the data
data_pix = reshape(data_pix,numCols,[])';

%Resize Data to get rid of first row
data=data(2:end,:);

%Convert P into covariances
data(:,15:23)=sqrt(data(:,15:23));
%Data [Filter Time, States, GPS_linux Time(s),Lat(m),Lon(m),Meters(up)]

%Set Data vals to vars
Filter_Time=data(:,1);
Pn_est=data(:,2);
Pe_est=data(:,3);
Grnd_Spd_Est=data(:,4);
Course_ang_Est=wrapTo360(data(:,5)*(180/pi));
Wn_est=data(:,6);
We_est=data(:,7);
Yaw_est=wrapTo360(data(:,8)*(180/pi));
Alt_est=data(:,9);
Alt_vv_est=data(:,10);
GPS_Time_UNIX=data(:,11);
Pn=data(:,12);
Pe=data(:,13);
Alt=data(:,14);
Heading=data(:,24);
Pn_est_cov=data(:,15);
Pe_est_cov=data(:,16);
Grnd_Spd_Est_cov=data(:,17);
Course_ang_Est_cov=data(:,18);
Wn_est_cov=data(:,19);
We_est_cov=data(:,20);
Yaw_est_cov=data(:,21);
Alt_est_cov=data(:,22);
Alt_vv_est_cov=data(:,23);
Ground_Speed=data(:,25);
%Vo data: time   dt   Vx   Vy   Vz
VO_Vx=data(:,28);
VO_Vy=data(:,29);
VO_Vz=data(:,30);
VO_Time=data(:,26);
VO_dt=data(:,27);
RPY_Time=data(:,31);
roll=radtodeg(data(:,32));
pitch=radtodeg(data(:,33));
yaw=radtodeg(data(:,34));
RPY_Time = RPY_Time - RPY_Time(1)



GPS_Vx = zeros(length(Pe),1);
GPS_Vy = zeros(length(Pe),1);
GPS_Vz = zeros(length(Pe),1);

 K=5;
 for i = K+5:K:length(Pe)-1
    
    if   abs(Pn(i)-Pn(i-K)) < 1*10^(-6)
        Delta_Pn = 0;
    else    
        Delta_Pn = Pn(i)-Pn(i-K);
    end
    
    if   abs(Pe(i)-Pe(i-K)) < 1*10^(-6)
        Delta_Pe = 0;
    else    
        Delta_Pe = Pe(i)-Pe(i-K);
    end
    
     if   abs(Alt(i)-Alt(i-K)) < 1*10^(-6)
        Delta_Alt = 0;
    else    
        Delta_Alt = Alt(i)-Alt(i-K);
    end
    
    GPS_dt = GPS_Time_UNIX(i)-GPS_Time_UNIX(i-K)
  
        for count = i-K:i
            GPS_Vx(count)=Delta_Pn/GPS_dt;%distance m / time sec
            GPS_Vy(count)=Delta_Pe/GPS_dt;
            GPS_Vz(count)=Delta_Alt/GPS_dt;
        end
 end
figure(50)
subplot(3,1,1)
plot(RPY_Time,roll,'g')
hold on

ylabel('Vx')
xlabel('VO = G|GPS = r')
hold off

subplot(3,1,2)
plot(RPY_Time,pitch,'g')
hold on

ylabel('Vy')
hold off

subplot(3,1,3)
plot(RPY_Time,yaw,'g')
hold on
ylabel('Vz')
hold off
title('RPY')
Time=Filter_Time-Filter_Time(1);
figure(FigNum)


subplot(3,1,1)
plot(Time,VO_Vx,'g')
hold on
plot(Time,GPS_Vx,'r')
ylabel('Vx')
xlabel('VO = G|GPS = r')
hold off

subplot(3,1,2)
plot(Time,VO_Vy,'g')
hold on
plot(Time,GPS_Vy,'r')
ylabel('Vy')
hold off

subplot(3,1,3)
Truth_Mag_I = sqrt(GPS_Vy.^2 + GPS_Vx.^2);
Cal_Mag_I = sqrt(VO_Vy.^2 + VO_Vx.^2);
plot(Time,Cal_Mag,'g')
hold on
plot(Time,Truth_Mag,'r')
ylabel('Magnitude')
hold off

suptitle(Title_Super)

clearvars -except Time Truth_Mag_I Cal_Mag_I Save_Name 
save (['DataVO/',Save_Name,'.mat'])