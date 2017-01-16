
close all
clear all

Files1 = {'oneloopmm'; 'oneloopmmR'; 'oneloopmmRR'; 'oneloopmmVo'; 'oneloopmmVoR'; 'oneloopmmVoRR';...
    'longloopmm'; 'longloopmmR'; 'longloopmmRR'; 'longloopmmVo'; 'longloopmmVoR'; 'longloopmmVoRR'};

for i = 1:length(Files1)
    Save_Name = char(Files1(i));

Fontsize = 12;

%Pull in data byte array for filter and convert
FigNum =1;
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
Course_ang_Est_cov=wrapTo360(data(:,18)*(180/pi));
Wn_est_cov=data(:,19);
We_est_cov=data(:,20);
Yaw_est_cov=wrapTo360(data(:,21)*(180/pi));
Alt_est_cov=data(:,22);
Alt_vv_est_cov=data(:,23);
Ground_Speed=data(:,25);


%Calculate Ground speeds
%  Ground_Speed_Cal = zeros(length(Pe),1);
%  L=10;
%  for i = L+5:L:length(Pe)-1
%     
%   if   abs(Pn(i)-Pn(i-L)) < 1*10^(-6)
%         Delta_Pn = 0;
%   else    
%      Delta_Pn = Pn(i)-Pn(i-L);
%   end
%    if   abs(Pe(i)-Pe(i-L)) < 1*10^(-6)
%       Delta_Pe = 0;
%   else    
%      Delta_Pe = Pe(i)-Pe(i-L);
%   end
%   
%   if GPS_Time_UNIX(i)-GPS_Time_UNIX(i-L) == 0
%       Ground_Speed_Cal(i) = 1;
%   else
%       for count = i-L:i
%       Ground_Speed_Cal(count)=sqrt((Delta_Pn)^2+(Delta_Pe)^2)/(GPS_Time_UNIX(i)-GPS_Time_UNIX(i-L));
%       end
%   end
%  end
%%
%Calculate Course Angle
Course_Ang_Cal = zeros(length(Pe),1);
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
  
        for count = i-K:i
            Course_ang = pi/2 - atan2(Delta_Pn,Delta_Pe);
            
            if Course_ang < 0
                Course_ang = Course_ang + 2*pi;
            end
            Course_Ang_Cal(count)=Course_ang;
        end
 end
 Course_Ang_Cal = Course_Ang_Cal*180/pi; 
 Course_Ang_Cal = wrapTo360(Course_Ang_Cal);
 
%Plots
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
A = figure('units','normalized','outerposition',[0 0 1 1])

%%%%%%%%%%%
%Error = Est -Truth
%Time Value
Time=(Filter_Time-Filter_Time(1))*.5;%error in time but run was ~67secs not 2x67. So quick fix

subplot(4,3,1)
plot(Time,Pn_est-Pn,'g')
hold on
plot(Time,Pn_est_cov,'r')
plot(Time,-Pn_est_cov,'r')
ylabel('Pn|Error(m)','FontSize',Fontsize,'FontWeight','bold')
xlabel('Time(s)')
hold off

subplot(4,3,4)
plot(Time,Pe_est-Pe,'g')
hold on
plot(Time,Pe_est_cov,'r')
plot(Time,-Pe_est_cov,'r')
ylabel('Pe|Error(m)','FontSize',Fontsize,'FontWeight','bold')
xlabel('Time(s)')
hold off

subplot(4,3,7)
plot(Time,Alt_est-Alt,'g')
hold on
plot(Time,Alt_est_cov,'r')
plot(Time,-Alt_est_cov,'r')
ylabel('Alt|Error(m)','FontSize',Fontsize,'FontWeight','bold')
xlabel('Time(s)')
hold off

subplot(4,3,10)
plot(Time,Grnd_Spd_Est-Ground_Speed,'g')
hold on
plot(Time,Grnd_Spd_Est_cov,'r')
plot(Time,-Grnd_Spd_Est_cov,'r')
ylabel('GrndSpeed|Error(m/s)','FontSize',Fontsize,'FontWeight','bold')
xlabel('Time(s)')
hold off


CourseAng_Error = Course_ang_Est(5:length(Course_Ang_Cal)-K)-Course_Ang_Cal(5:end-K);
for i= 1:length(CourseAng_Error)
    if CourseAng_Error(i) > 180
        CourseAng_Error(i) = 360 - CourseAng_Error(i);
    end
    if CourseAng_Error(i) < -180
        CourseAng_Error(i) = 360 + CourseAng_Error(i);
    end
end
subplot(4,3,2)
plot(Time(5:length(Course_Ang_Cal)-K),CourseAng_Error,'g')
hold on
plot(Time,Course_ang_Est_cov,'r')
plot(Time,-Course_ang_Est_cov,'r')
ylabel('CourseAng|Error(deg)','FontSize',Fontsize,'FontWeight','bold')
xlabel('Time(s)')
hold off

subplot(4,3,5)
plot(Time,Wn_est,'g')
hold on
plot(Time,Wn_est_cov+Wn_est,'r')
plot(Time,-Wn_est_cov+Wn_est,'r')
ylabel('WindNorth(m/s)','FontSize',Fontsize,'FontWeight','bold')
xlabel('Time(s)')
hold off

subplot(4,3,8)
plot(Time,We_est,'g')
hold on
plot(Time,We_est_cov+We_est,'r')
plot(Time,-We_est_cov+We_est,'r')
ylabel('WindEast(m/s)','FontSize',Fontsize,'FontWeight','bold')
xlabel('Time(s)')
hold off

% subplot(4,3,11)
% plot(Time,Yaw_est,'g')
% hold on
% plot(data_pix(2:end,5)-Filter_Time(1),data_pix(2:end,6))
% plot(Time,Yaw_est_cov,'r')
% plot(Time,-Yaw_est_cov,'r')
% ylabel('Yaw')
% hold off

Yaw_Error = Yaw_est-Heading;
for i= 1:length(Yaw_Error)
    if Yaw_Error(i) > 180
        Yaw_Error(i) = 360 - Yaw_Error(i);
    end
    if Yaw_Error(i) < -180
        Yaw_Error(i) = 360 + Yaw_Error(i);
    end
end

subplot(4,3,11)
plot(Time,Yaw_Error,'g')
hold on
plot(Time,Yaw_est_cov,'r')
plot(Time,-Yaw_est_cov,'r')
ylabel('Yaw|Error(deg)','FontSize',Fontsize,'FontWeight','bold')
xlabel('Time(s)')
hold off

subplot(4,3,3)
ALT_est = plot(Time,Alt_vv_est,'g')
hold on
ALT_cov = plot(Time,Alt_est_cov,'r')
plot(Time,-Alt_est_cov,'r')
ylabel('AltVV(m/s)','FontSize',Fontsize,'FontWeight','bold')
xlabel('Time(s)')
hold off

% subplot(4,3,6)
% plot(Time,TWO_DRMS,'g')
% hold on
% 
% ylabel('2DRMS(m)','FontSize',Fontsize,'FontWeight','bold')
% xlabel('Time(s)')
% hold off

%Overhead
subplot(4,3,[6,9,12])
Truth_pos = plot(data(:,13),data(:,12),'b')
hold on
plot(data(:,3),data(:,2),'g')
axis equal
xlabel('East(m)','FontSize',Fontsize,'FontWeight','bold')
ylabel('North(m)','FontSize',Fontsize,'FontWeight','bold')

legend = legend([Truth_pos,ALT_est,ALT_cov],'Truth 2D Position','Estimate','1-\sigma Covariance','Location','southeast')
legend.FontSize = 12;
%suptitle(Title_Super)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

saveas(gcf,['Figures/',Save_Name],'epsc')
savefig(['Figures/',Save_Name])

North_sigma = std(Pn_est-Pn);
East_sigma = std(Pe_est-Pe);
TWO_DRMS = 2*sqrt(North_sigma^2+East_sigma^2);

clearvars -except North_sigma East_sigma TWO_DRMS Save_Name Files1
save (['Figures/',Save_Name,'.mat'])
pause(1)
close all
end