filename= '/home/suas/IdeaProjects/MotionModel/Scorpion_Fain/Filter_Output';
fid = fopen(filename, 'r');
data = fread(fid,'double',0,'b');
fclose(fid);
numCols = 10;
data = reshape(data,numCols,[])';
