filename= '/home/suas/IdeaProjects/MotionModel/Scorpion_Fain/Filter_Output/SampleRun.txt';
fid = fopen(filename, 'r');
data = fread(fid,'double',0,'b');
fclose(fid);
numCols = 10;
data = reshape(data,numCols,[])';
data_deg = data;
data_deg(:,5)=data(:,5)*(180/pi);
data_deg(:,8)=data(:,8)*(180/pi);

figure(1)
plot(data(:,3),data(:,2),'X')
figure(2)
plot(data(:,3),data(:,2),'X')
axis equal