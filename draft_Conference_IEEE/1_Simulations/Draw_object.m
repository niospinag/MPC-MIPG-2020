function Draw_object(vhist,zhist,vphist,zphist,dhist,T)
% Draw_MIPG(vhist,zhist,...
%                             vp1hist,zp1hist,...
%                             vp2hist,zp2hist,...
%                                                 dhist,T,N)
%        Draw_MIPG(vhist,vp1hist,zhist,zp1hist,dhist,T,N)

N = size(vphist,2)-1;%Horizonte de prediccion
V = size(vphist,3);%numero de vehiculos

vp1hist = [];

set(0,'DefaultAxesFontName', 'Times New Roman')
set(0,'DefaultAxesFontSize', 12)
% zphist=[ones(1,size(zphist,2))*zhist(1,1);zphist];
line_width = 1.5;
fontsize_labels = 14;
% V
%--------------------------------------------------------------------------
%-----------------------Simulate robots -----------------------------------
%--------------------------------------------------------------------------
an = 10; alt=0.5; % parametros carroceria

rob_diam=1.5;
r = rob_diam/2;  % robot radius
ang=0:0.005:2*pi;
x_circle=an*cos(ang);
y_circle=alt*sin(ang);
% 
% r = obs_diam/2;  % obstacle radius
% xp_obs=r*cos(ang);
% yp_obs=r*sin(ang);


figure(500)%create a window named "500"
% Animate the robot motion
set(gcf,'PaperPositionMode','auto')
set(gcf, 'Color', 'w');
set(gcf,'Units','normalized','OuterPosition',[0 0 1 0.5]);%tamaño del grafico
% 
% xp_1=[T*vphist(1,:)];
% xp_2=dhist(1:(size(vphist,2)));
zph_3=ones(81,size(vp1hist,2))*zhist(3);
zph_4=ones(81,size(vp1hist,2))*zhist(4);
% for k = 1:size(vphist,1)
% xp_1(k+1,:)=xp_1(k,:)+T*vphist(k,:);
% xp_2(k+1,:)=xp_2(k,:)+T*15;
% end
% filename = '2c_1obs.gif';% <---------------------------------------------------------------------------------------------------------------
xp=zeros(1,size(vphist,3)+1,V);


for n=2:V
    xp(1,1,n)=dhist(n-1);
end



 % ---------vehiculo n--------
for n = 1:size(vphist,3)
    for j = 1:size(vphist,1)
        xp(j+1,1,n)=xp(j,1,n)+T*vhist(n,j);
        for i = 1:(size(vphist,2)-1)
            xp(j,i+1,n)=xp(j,i,n)+T*vphist(j,i,n);
        end
    end
end








x=zeros(V,size(vhist,2));

for n=2:V
    x(1,1,n)=dhist(n-1);
end



%  dibujo dinamico
for k = 1:size(vhist,2)
   
    for n=1:V
    x(1,k+1,n)=x(1,k,n)+T*vhist(n,k);
    end
    y=zhist(:,k);
%-------------Plot any car----------------    
plot_car(an,alt,x(1,k,1),y(1),0,'b')%Plot the car
plot_car(an,alt,x(1,k,2),y(2),0,'r')%Plot the car
plot_car(an,alt,x(1,k,3),y(3),0,'g')%Plot the car
plot_car(an,alt,x(1,k,4),y(4),0,'c')%Plot the car
 


hold on;
    
   


%-----------Plot trajectories------------    
%------------agente 1------------
    plot(x(1,k,1),y(2),'-r','linewidth',line_width);hold on % plot exhibited trajectory
    if k < size(vhist,2) % plot prediction
        plot(xp(k,:,1),zphist(k,:,1),'b--*')
    end
    plot(x(1,k,1)+x_circle,y(1)+y_circle,'--b')% plot robot circle
%------------agente 2------------
    plot(x(1,k,2),y(2),'-r','linewidth',line_width);hold on % plot exhibited trajectory
    if k < size(vhist,2) % plot prediction
        plot(xp(k,:,2),zphist(k,:,2),'r--*')
    end
    plot(x(1,k,2)+x_circle,y(2)+y_circle,'--r')% plot robot circle
%------------agente 3------------
    plot(x(1,k,3),y(3),'-r','linewidth',line_width);hold on % plot exhibited trajectory
    if k < size(vhist,2) % plot prediction
        plot(xp(k,:,3),zphist(k,:,3),'g--*')
    end
    plot(x(1,k,3)+x_circle,y(3)+y_circle,'--g')% plot robot circle
    
%------------agente 4------------
    plot(x(1,k,4),y(4),'-r','linewidth',line_width);hold on % plot exhibited trajectory
    if k < size(vhist,2) % plot prediction
        plot(xp(k,:,4),zphist(k,:,4),'c--*')
    end
    plot(x(1,k,4)+x_circle,y(4)+y_circle,'--c')% plot robot circle
    
    
    
    
    
   hold off
    %figure(500)
    ylabel('$y$-position (m)','interpreter','latex','FontSize',fontsize_labels)
    xlabel('$x$-position (m)','interpreter','latex','FontSize',fontsize_labels)
    axis([-0.2 400 -0.2 6])%Axis description
    pause(0.3)
    box on;
    grid on
    %aviobj = addframe(aviobj,gcf);
    drawnow
    % for video generation
    F(k) = getframe(gcf); % to get the current frame
    
    
    %---------------------make gift--------------------------
% frame = getframe(figure(500)); 
%       im = frame2im(frame); 
%       [imind,cm] = rgb2ind(im,256); 
%       % Write to the GIF File 
%       if k == 1 
%           imwrite(imind,cm,filename,'gif', 'Loopcount',inf); 
%       else 
%           imwrite(imind,cm,filename,'gif','WriteMode','append'); 
%       end 
% 
%     
    
end
close(gcf)