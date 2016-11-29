filename= '/home/suas/IdeaProjects/MotionModel/Scorpion_Fain/Filter_Output/SampleRun.txt';
fid = fopen(filename, 'r');
data = fread(fid,'double',0,'b');
fclose(fid);
numCols = 10;
data = reshape(data,numCols,[])';
figure(2)
plot(data(:,3),data(:,2),'X')