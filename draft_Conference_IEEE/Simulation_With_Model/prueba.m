time=200; %tiempo de procesamiento
vel= zeros(2,time);% velociodad inicial [vy vx]
Vdes=[20]; %velocidad deseada
t=0.02;
angle = zeros(2,time); % angulo inicial [fi dfi]
ang_des = [0]; %angulo deseado
pos = zeros(2,time); %posicion inicial
ydes = [3]; %carril deseado
m=1;
lf=1; %distance from center to front
lr=1; %distance from center to rear
iz = 1; %Inertia coefficient


posh=[];
velh=[];
angleh=[];
fyf = -0.1;
fyr = 0.1;
fxf = 1;
fxr = 1;


for i=1:199
     vel(2,i+1) = vel(2,i) - t*angle(2,i)*vel(1,i) - t*vel(1,i)*angle(2,i) + (2/m)*fyf*t + (2/m)*fyr*t ;
     vel(1,i+1) = vel(1,i) + t*angle(2,i)*vel(2,i) + t*vel(2,i)*angle(2,i) + (2/m)*fxf*t + (2/m)*fxr*t ;
     angle(1,i+1) = angle(1,i) + t*angle(2,i);
     angle(2,i+1) = angle(2,i) + (2*lf/iz)*t*fyf - (2*lr/iz)*t*fyr;
     pos(2,i+1) = t*cos(angle(1,i))*vel(2,i) + t*sin(angle(1,i))*vel(1,i) + angle(1)*vel(1,i)*cos(angle(1,i))*t - angle(1,i)*vel(2,i)*sin(angle(1,i))*t + pos(2,i);
     pos(1,i+1) = -t*sin(angle(1,i))*vel(2,i) - t*cos(angle(1,i))*vel(1,i) - angle(1)*vel(1,i)*sin(angle(1,i))*t - angle(1,i)*vel(2,i)*cos(angle(1,i))*t + pos(1,i);
end

subplot(1,3,1)
plot(pos(1,:),pos(2,:))
title('Position')

subplot(1,3,2)
plot(vel(1,:),vel(2,:))
title('Velocity')

subplot(1,3,3)
plot(angle(1,:),angle(2,:))
title('Angle')