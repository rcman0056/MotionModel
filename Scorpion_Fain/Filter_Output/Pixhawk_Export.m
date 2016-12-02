filename= '/home/suas/IdeaProjects/MotionModel/Scorpion_Fain/Filter_Output/Pixhawk_Data.txt';
fid = fopen(filename, 'r');
data = fread(fid,'double',0,'b');
fclose(fid);
numCols = 6;
Pixhawk_Data = reshape(data,numCols,[])';
[row,col] = size(Pixhawk_Data);
Pixhawk_Data = Pixhawk_Data(2:row,1:col);
Description = '[Mag Time(s),xmag(milli-tesla),ymag(milli-tesla),zmag(milli-tesla),Heading Time(s),Heading(deg)]';

save('Pixhawk_Data_Raw.mat', 'Pixhawk_Data', 'Description' )