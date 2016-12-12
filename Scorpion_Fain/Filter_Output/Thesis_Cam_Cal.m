addpath('C:\Users\ANT\Google Drive\AFIT\Research\CAM_CAL_PHOTOS\toolbox_calib\TOOLBOX_calib') 
%addpath('C:\Users\Benjamin Fain\Google Drive\AFIT\Research\CAM_CAL_PHOTOS\toolbox_calib\TOOLBOX_calib') 
%calib_gui


%Calib_Results_01.mat
% Focal Length:          fc = [ 1026.91452   1024.06770 ] +/- [ 11.10437   11.76917 ]
% Principal point:       cc = [ 679.00481   463.82720 ] +/- [ 8.03716   8.70406 ]
% Skew:             alpha_c = [ 0.00000 ] +/- [ 0.00000  ]   => angle of pixel axes = 90.00000 +/- 0.00000 degrees
% Distortion:            kc = [ -0.39374   0.23685   -0.00009   -0.00033  0.00000 ] +/- [ 0.01264   0.02225   0.00122   0.00135  0.00000 ]
% Pixel error:          err = [ 1.47572   1.04269 ]

%After running recommendation of not including zeros
%Calib_Results_02.mat
% Focal Length:          fc = [ 1026.30894   1023.33181 ] +/- [ 10.89177   11.39588 ]
% Principal point:       cc = [ 678.01528   463.49804 ] +/- [ 7.40620   7.37536 ]
% Skew:             alpha_c = [ 0.00000 ] +/- [ 0.00000  ]   => angle of pixel axes = 90.00000 +/- 0.00000 degrees
% Distortion:            kc = [ -0.39187   0.23479   -0.00000   -0.00000  0.00000 ] +/- [ 0.01096   0.02139   0.00000   0.00000  0.00000 ]
% Pixel error:          err = [ 1.47636   1.04194 ]

%%%%%%%%%%%%%%New click

% Focal Length:          fc = [ 998.09342   1005.01966 ] +/- [ 8.58851   9.09444 ]
% Principal point:       cc = [ 670.90144   466.79380 ] +/- [ 6.16453   5.98787 ]
% Skew:             alpha_c = [ 0.00000 ] +/- [ 0.00000  ]   => angle of pixel axes = 90.00000 +/- 0.00000 degrees
% Distortion:            kc = [ -0.37030   0.18955   -0.00000   -0.00000  0.00000 ] +/- [ 0.00839   0.01510   0.00000   0.00000  0.00000 ]
% Pixel error:          err = [ 1.25263   0.75193 ]

fc = [ 998.09342   1005.01966 ];
cc = [ 670.90144   466.79380 ];
kc = [ -0.37030   0.18955   -0.00000   -0.00000  0.00000 ];
alpha_c = [ 0.00000 ];
x_kk=[1 100]';
[xn] = normalize(x_kk,fc,cc,kc,alpha_c)