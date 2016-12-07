
close all
clear all

%Pull in data byte array and convery
filename= '/home/suas/IdeaProjects/MotionModel/Scorpion_Fain/Filter_Output/SampleRun.txt';
fid = fopen(filename, 'r');
data = fread(fid,'double',0,'b');
fclose(fid);
numCols = 14; %Set num of cols in data bytearray so matlab can parse the data
data = reshape(data,numCols,[])';
data_deg = data;
data_deg(:,5)=data(:,5)*(180/pi);
data_deg(:,8)=data(:,8)*(180/pi);


%Resize Data to get rid of first row
data=data(2:end,:);
%Data [Filter Time, States, GPS_linux Time(s),Lat(m),Lon(m),Meters(up)]

%Set Data vals to vars
Filter_Time=data(:,1);
Pn_est=data(:,2);
Pe_est=data(:,3);
Grnd_Spd_Est=data(:,4);
Course_ang_Est=data(:,5);
Wn_est=data(:,6);
We_est=data(:,7);
Yaw_est=data(:,8);
Alt_est=data(:,9);
Alt_vv_est=data(:,10);
GPS_unix_time=data(:,11);
Pn=data(:,12);
Pe=data(:,13);
Alt=data(:,14);

%Plots
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(1)
%%%%%%%%%%%
%Error = Est -Truth

subplot(3,2,1)
plot(Filter_Time-Filter_Time(1),Pn_est-Pn,'g')
ylabel('Pn|Error')

subplot(3,2,3)
plot(Filter_Time-Filter_Time(1),Pe_est-Pe,'g')
ylabel('Pe|Error')

subplot(3,2,5)
plot(Filter_Time-Filter_Time(1),Alt_est-Alt,'g')
ylabel('Alt|Error')


%Overhead
subplot(3,2,[2,4,6])
plot(data(:,13),data(:,12),'r')
hold on
plot(data(:,3),data(:,2),'g')
axis equal
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



