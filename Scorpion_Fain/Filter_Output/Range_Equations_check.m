%Range Equations Check
clear all
CenterX = -200;
CenterY = 200;
dt = .107357;
Speed = 30;
R = 300;
PositionX=CenterX;
PositionY=CenterY+R;
dt_total = 0;
figure(1)
axis equal
for i = 1:200

    if dt_total == 0 
    dt_total = dt;
    end
    
Angle_diff=-(dt_total*Speed)/(2*pi*R); %add the pi/2 to get a plane offset

PositionX_New(i)=cos(Angle_diff)*R+CenterX

PositionY_New(i)=sin(Angle_diff)*R+CenterY
dt_total=dt_total+dt;

plot(PositionX_New,PositionY_New,'x')
pause(.5)
end

