
%close all
%clear all
Save_Name = 'OneLoopMM';
Title_Super =['Qdmm = [.5,3,3,1,1] and  Run =' Save_Name];
FigNum=2;
%Pull in data byte array for filter and convert

filename= ['/home/suas/IdeaProjects/MotionModel/Scorpion_Fain/Filter_Output/SampleRun_' Save_Name '.txt'];
fid = fopen(filename, 'r');
data = fread(fid,'double',0,'b');
fclose(fid);
numCols = 24; %Set num of cols in data bytearray so matlab can parse the data
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
%Calculate Ground speeds
 Ground_Speed_Cal = zeros(length(Pe),1);
 L=10;
 for i = L+5:L:length(Pe)-1
    
  if   abs(Pn(i)-Pn(i-L)) < 1*10^(-6)
        Delta_Pn = 0;
  else    
     Delta_Pn = Pn(i)-Pn(i-L);
  end
   if   abs(Pe(i)-Pe(i-L)) < 1*10^(-6)
      Delta_Pe = 0;
  else    
     Delta_Pe = Pe(i)-Pe(i-L);
  end
  
  if GPS_Time_UNIX(i)-GPS_Time_UNIX(i-L) == 0
      Ground_Speed_Cal(i) = 1;
  else
      for count = i-L:i
      Ground_Speed_Cal(count)=sqrt((Delta_Pn)^2+(Delta_Pe)^2)/(GPS_Time_UNIX(i)-GPS_Time_UNIX(i-L));
      end
  end
 end
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
            Course_ang = atan2(Delta_Pn,Delta_Pe);
            
            if Course_ang < 0
                Course_ang = -Course_ang + pi/2;
            
            elseif Course_ang*(180/pi) == 180.0 
                Course_ang = 1.5*pi;
            else
                    if Delta_Pe < 0
                        Course_ang = 2*pi-(Course_ang-pi/2);
                    end
                    if Delta_Pe > 0
                        Course_ang = pi - Course_ang;
                    end
             end
            
        
        
            
            Course_Ang_Cal(count)=Course_ang;
        end
 end
 Course_Ang_Cal = Course_Ang_Cal*180/pi; 
 Course_Ang_Cal = wrapTo360(Course_Ang_Cal);
 
%Plots
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(FigNum)
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
plot(Time(5:length(Ground_Speed_Cal)-L),Grnd_Spd_Est(5:length(Ground_Speed_Cal)-L)-Ground_Speed_Cal(5:end-L),'g')
%plot(Time,Grnd_Spd_Est,'g')
hold on
plot(Time,Grnd_Spd_Est_cov,'r')
plot(Time,-Grnd_Spd_Est_cov,'r')
ylabel('GrndSpeed|Error')
hold off

subplot(4,3,2)
plot(Time(5:length(Course_Ang_Cal)-K),Course_ang_Est(5:length(Course_Ang_Cal)-K)-Course_Ang_Cal(5:end-K),'g')
hold on
plot(Time,Course_ang_Est_cov,'r')
plot(Time,-Course_ang_Est_cov,'r')
ylabel('CourseAng(deg)|Error')
hold off

subplot(4,3,5)
plot(Time,Wn_est,'g')
hold on
plot(Time,Wn_est_cov+Wn_est,'r')
plot(Time,-Wn_est_cov+Wn_est,'r')
ylabel('WindNorth')
hold off

subplot(4,3,8)
plot(Time,We_est,'g')
hold on
plot(Time,We_est_cov+We_est,'r')
plot(Time,-We_est_cov+We_est,'r')
ylabel('WindEast')
hold off

subplot(4,3,11)
plot(Time,Yaw_est,'g')
hold on
plot(data_pix(2:end,5)-Filter_Time(1),data_pix(2:end,6))
plot(Time,Yaw_est_cov,'r')
plot(Time,-Yaw_est_cov,'r')
ylabel('Yaw')
hold off

subplot(4,3,6)
plot(Time,Yaw_est-Heading,'g')
hold on
plot(Time,Yaw_est_cov,'r')
plot(Time,-Yaw_est_cov,'r')
ylabel('Yaw|Error')
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

suptitle(Title_Super)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

