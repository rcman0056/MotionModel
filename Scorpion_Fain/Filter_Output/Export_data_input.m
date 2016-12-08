
close all
clear all

%Pull in data byte array and convert
filename= '/home/suas/IdeaProjects/MotionModel/Scorpion_Fain/Filter_Output/SampleRun.txt';
fid = fopen(filename, 'r');
data = fread(fid,'double',0,'b');
fclose(fid);
numCols = 23; %Set num of cols in data bytearray so matlab can parse the data
data = reshape(data,numCols,[])';
 


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
Course_ang_Est=data(:,5)*(180/pi);
Wn_est=data(:,6);
We_est=data(:,7);
Yaw_est=data(:,8)*(180/pi);
Alt_est=data(:,9);
Alt_vv_est=data(:,10);
GPS_unix_time=data(:,11);
Pn=data(:,12);
Pe=data(:,13);
Alt=data(:,14);
Pn_est_cov=data(:,15);
Pe_est_cov=data(:,16);
Grnd_Spd_Est_cov=data(:,17);
Course_ang_Est_cov=data(:,18);
Wn_est_cov=data(:,19);
We_est_cov=data(:,20);
Yaw_est_cov=data(:,21);
Alt_est_cov=data(:,22);
Alt_vv_est_cov=data(:,23);
%Plots
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(1)
%%%%%%%%%%%
%Error = Est -Truth
%Time Value
Time=Filter_Time-Filter_Time(1);

subplot(4,3,1)
plot(Time,Pn_est-Pn,'g')
hold on
plot(Time,Pn_est_cov,'r')
plot(Time,-Pn_est_cov,'r')
ylabel('Pn|Error')
hold off

subplot(4,3,4)
plot(Time,Pe_est-Pe,'g')
hold on
plot(Time,Pe_est_cov,'r')
plot(Time,-Pe_est_cov,'r')
ylabel('Pe|Error')
hold off

subplot(4,3,7)
plot(Time,Alt_est-Alt,'g')
hold on
plot(Time,Alt_est_cov,'r')
plot(Time,-Alt_est_cov,'r')
ylabel('Alt|Error')
hold off

subplot(4,3,10)
plot(Time,Grnd_Spd_Est,'g')
hold on
plot(Time,Grnd_Spd_Est_cov,'r')
plot(Time,-Grnd_Spd_Est_cov,'r')
ylabel('GrndSpeed')
hold off

subplot(4,3,2)
plot(Time,Course_ang_Est,'g')
hold on
plot(Time,Course_ang_Est_cov,'r')
plot(Time,-Course_ang_Est_cov,'r')
ylabel('CourseAng(deg)')
hold off

subplot(4,3,5)
plot(Time,Wn_est,'g')
hold on
plot(Time,Wn_est_cov,'r')
plot(Time,-Wn_est_cov,'r')
ylabel('WindNorth')
hold off

subplot(4,3,8)
plot(Time,We_est,'g')
hold on
plot(Time,We_est_cov,'r')
plot(Time,-We_est_cov,'r')
ylabel('WindEast')
hold off

subplot(4,3,11)
plot(Time,Yaw_est,'g')
hold on
plot(Time,Yaw_est_cov,'r')
plot(Time,-Yaw_est_cov,'r')
ylabel('Yaw')
hold off

subplot(4,3,3)
plot(Time,Alt_vv_est,'g')
hold on
plot(Time,Alt_est_cov,'r')
plot(Time,-Alt_est_cov,'r')
ylabel('AltVV')
hold off

%Overhead
subplot(4,3,[9,12])
plot(data(:,13),data(:,12),'r')
hold on
plot(data(:,3),data(:,2),'g')
axis equal
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



