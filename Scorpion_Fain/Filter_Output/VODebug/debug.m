clear
%close all
DEBUG='/home/suas/IdeaProjects/MotionModel/Scorpion_Fain/Filter_Output/VODebug/debug.txt';
fid = fopen(DEBUG);
rawData = textscan(fid,'%s');
fclose(fid);

numDebugTypes = 15;

data = cell(numDebugTypes,1);
for i=1:numDebugTypes
    data{i} = cell(1);
end
index = ones(numDebugTypes,2);
storeData = 0;
for i = 1:length(rawData{1})
    tempText = rawData{1}(i);
    
    if length(tempText{1})>2
        if strcmp(tempText{1}(1:3), 'END')
            storeData = 0;
            index(debugNum+1,1) = index(debugNum+1,1)+1;
            index(debugNum+1,2) = 1;
        end
    end
    
    if storeData
        data{debugNum+1}(index(debugNum+1,1),index(debugNum+1,2)) = tempText;
        index(debugNum+1,2) = index(debugNum+1,2)+1;
    end
    if length(tempText{1})>8
        if strcmp(tempText{1}(1:9), 'DEBUGGING')
            storeData = 1;
            debugNum = str2double(tempText{1}(10:11));
        end
    end
    
end
outliers = ~strcmp(data{11,1}(:,1), '0');
% pixel = cellfun(@str2double,data{1}(:,1:4));
% pixelT = pixel(:,3:4)-pixel(:,1:2);
% figure
% title('Pixel Coordinates')
% subplot(2,1,1)
% plot(pixelT(:,1))
% subplot(2,1,2)
% plot(pixelT(:,2))

debugPlots( data, 1, 2, 'Pixel Coordinates Translation' ,outliers)
debugPlots( data, 2, 3, 'Normalized Coordinates Translation' ,outliers)
debugPlots( data, 6, 3, 'Camera Coordinates Translation' ,outliers)
debugPlots( data, 7, 3, 'Scaled NED Coordinates Translation' ,outliers)

%calculate RPY from DCM's
firstCamToNavDcm = zeros(3,3,size(data{9},1));
firstCamToNavDcm(1,:,:) = cellfun(@str2double,data{9}(:,1:3))';
firstCamToNavDcm(2,:,:) = cellfun(@str2double,data{9}(:,4:6))';
firstCamToNavDcm(3,:,:) = cellfun(@str2double,data{9}(:,7:9))';
firstCamToNav = DcmToRpy(firstCamToNavDcm)';

secondCamToNavDcm = zeros(3,3,size(data{10},1));
secondCamToNavDcm(1,:,:) = cellfun(@str2double,data{10}(:,1:3))';
secondCamToNavDcm(2,:,:) = cellfun(@str2double,data{10}(:,4:6))';
secondCamToNavDcm(3,:,:) = cellfun(@str2double,data{10}(:,7:9))';
secondCamToNav = DcmToRpy(secondCamToNavDcm)';

firstCamToSecondCamDcm = zeros(3,3,size(data{13},1));
firstCamToSecondCamDcm(1,:,:) = cellfun(@str2double,data{13}(:,1:3))';
firstCamToSecondCamDcm(2,:,:) = cellfun(@str2double,data{13}(:,4:6))';
firstCamToSecondCamDcm(3,:,:) = cellfun(@str2double,data{13}(:,7:9))';
firstCamToSecondCam = DcmToRpy(firstCamToSecondCamDcm)';

firstCamToSecondCamDcm2 = zeros(3,3,size(data{14},1));
firstCamToSecondCamDcm2(1,:,:) = cellfun(@str2double,data{14}(:,1:3))';
firstCamToSecondCamDcm2(2,:,:) = cellfun(@str2double,data{14}(:,4:6))';
firstCamToSecondCamDcm2(3,:,:) = cellfun(@str2double,data{14}(:,7:9))';
firstCamToSecondCam2 = DcmToRpy(firstCamToSecondCamDcm2)';

firstCamToSecondCamDcm3 = zeros(3,3,size(data{15},1));
firstCamToSecondCamDcm3(1,:,:) = cellfun(@str2double,data{15}(:,1:3))';
firstCamToSecondCamDcm3(2,:,:) = cellfun(@str2double,data{15}(:,4:6))';
firstCamToSecondCamDcm3(3,:,:) = cellfun(@str2double,data{15}(:,7:9))';
firstCamToSecondCam3 = DcmToRpy(firstCamToSecondCamDcm3)';

firstCamToSecondCam4 = zeros(size(firstCamToSecondCam));
for i = 1:length(firstCamToSecondCamDcm)
    if firstCamToSecondCamDcm(1,1,i) > 0
        firstCamToSecondCam4(i,:) = firstCamToSecondCam(i,:);
    else
        firstCamToSecondCam4(i,:) = firstCamToSecondCam3(i,:);
    end
end

%plot final translation
%translation = zeros(3,size(data{12},1));
translation = cellfun(@str2double,data{12}(:,1:3))';
numOutliers = cellfun(@str2double,data{11}(:,2))';
figure
subplot(3,1,1)
plot(translation(1,:))
hold on
plot(numOutliers)
subplot(3,1,2)
plot(translation(2,:))
subplot(3,1,3)
plot(translation(3,:))

% %plot attitudes's
% figure
% subplot(3,1,1)
% plot(firstCamToNav(:,1))
% title('Attitude of camera relative to NED for the first image')
% subplot(3,1,2)
% plot(firstCamToNav(:,2))
% subplot(3,1,3)
% plot(firstCamToNav(:,3))
%
% figure
% subplot(3,1,1)
% plot(secondCamToNav(:,1))
% title('Attitude of camera relative to NED for the second image')
% subplot(3,1,2)
% plot(secondCamToNav(:,2))
% subplot(3,1,3)
% plot(secondCamToNav(:,3))

for i = 1:size(secondCamToNavDcm,3)
    for j = 1:3
        for k = 1:3
            term1 = secondCamToNavDcm(j,1,i) * firstCamToSecondCamDcm(1, k,i);
            term2 = secondCamToNavDcm(j,2,i) * firstCamToSecondCamDcm(2, k,i);
            term3 = secondCamToNavDcm(j,3,i) * firstCamToSecondCamDcm(3, k,i);
            firstCamToNavDcm(j,k,i) = term1 + term2 + term3;
        end
    end
    firstCamToNavDcm2(:,:,i) = secondCamToNavDcm(:,:,i)*firstCamToSecondCamDcm(:,:,i);
end
firstCamToNavCalc = DcmToRpy(firstCamToNavDcm)';
firstCamToNavCalc2 = DcmToRpy(firstCamToNavDcm2)';

%plot attitudes's
figure
xStuff = 1:length(firstCamToNav)+1;
subplot(3,1,1)
plot(xStuff(1:end-1),firstCamToNav(1:end,1),'LineWidth',3)
hold on
plot(xStuff(2:end),secondCamToNav(:,1),'LineWidth',1)
plot(xStuff(1:end-1),firstCamToNavCalc(1:end,1),'LineWidth',3)
plot(xStuff(1:end-1),firstCamToNavCalc2(1:end,1),'LineWidth',1)
title('Attitude of camera relative to NED')
legend('First','Second', 'First Calc','First Calc2')
subplot(3,1,2)
plot(xStuff(1:end-1),firstCamToNav(1:end,2),'LineWidth',3)
hold on
plot(xStuff(2:end),secondCamToNav(:,2),'LineWidth',1)
plot(xStuff(1:end-1),firstCamToNavCalc(1:end,2),'LineWidth',3)
plot(xStuff(1:end-1),firstCamToNavCalc2(1:end,2),'LineWidth',1)
subplot(3,1,3)
plot(xStuff(1:end-1),firstCamToNav(1:end,3),'LineWidth',3)
hold on
plot(xStuff(2:end),secondCamToNav(:,3),'LineWidth',1)
plot(xStuff(1:end-1),firstCamToNavCalc(1:end,3),'LineWidth',3)
plot(xStuff(1:end-1),firstCamToNavCalc2(1:end,3),'LineWidth',1)

figure
xStuff = 1:length(firstCamToSecondCam);
subplot(3,1,1)
plot(xStuff,firstCamToSecondCam(:,1),'LineWidth',4)
hold on
plot(xStuff,firstCamToSecondCam2(:,1),'LineWidth',3)
plot(xStuff,firstCamToSecondCam3(:,1),'LineWidth',2)
%plot(xStuff,firstCamToSecondCam4(:,1),'LineWidth',1)
title('Attitude of first camera relative to second')
legend('R1','R', 'R2')
subplot(3,1,2)
plot(xStuff,firstCamToSecondCam(:,2),'LineWidth',4)
hold on
plot(xStuff,firstCamToSecondCam2(:,2),'LineWidth',3)
plot(xStuff,firstCamToSecondCam3(:,2),'LineWidth',2)
%plot(xStuff,firstCamToSecondCam4(:,2),'LineWidth',1)
subplot(3,1,3)
plot(xStuff,firstCamToSecondCam(:,3),'LineWidth',4)
hold on
plot(xStuff,firstCamToSecondCam2(:,3),'LineWidth',3)
plot(xStuff,firstCamToSecondCam3(:,3),'LineWidth',2)
%plot(xStuff,firstCamToSecondCam4(:,3),'LineWidth',1)

figure
xStuff = 1:length(firstCamToSecondCam);
subplot(3,1,1)
plot(xStuff,firstCamToSecondCam2(:,1),'LineWidth',3)
title('Attitude of first camera relative to second')
legend('R')
subplot(3,1,2)
hold on
plot(xStuff,firstCamToSecondCam2(:,2),'LineWidth',3)
subplot(3,1,3)
plot(xStuff,firstCamToSecondCam2(:,3),'LineWidth',3)



