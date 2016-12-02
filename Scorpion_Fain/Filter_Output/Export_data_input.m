filename= '/home/suas/IdeaProjects/MotionModel/Scorpion_Fain/Filter_Output/SampleRun.txt';
fid = fopen(filename, 'r');
data = fread(fid,'double',0,'b');
fclose(fid);
numCols = 14;
data = reshape(data,numCols,[])';
data_deg = data;
data_deg(:,5)=data(:,5)*(180/pi);
data_deg(:,8)=data(:,8)*(180/pi);


figure(1)
plot(data(:,3),data(:,2),'X')
axis equal
title('Flight Path')

%Data [Filter Time, States, GPS_linux Time,Lat(m),Lon(m),AGL]
subplot(3,3,1)
plot(data(:,1),data(:,2),'r')
Hold on
plot(data(:,
axis equal


