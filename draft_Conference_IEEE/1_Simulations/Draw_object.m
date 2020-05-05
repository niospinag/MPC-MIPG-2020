function Draw_object(vhist,zhist,vphist,zphist,dhist,T)
% Draw_MIPG(vhist,zhist,...
%                             vp1hist,zp1hist,...
%                             vp2hist,zp2hist,...
%                                                 dhist,T,N)
%        Draw_MIPG(vhist,vp1hist,zhist,zp1hist,dhist,T,N)

N = size(vphist,2)-1;
V = size(vphist,3);

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
xp=an*cos(ang);
yp=alt*sin(ang);
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
xp=zeros(1,size(vphist,3),3);

xp_2(1,1)=[dhist(1)];
xp_3(1,1)=[dhist(2)];


 % ---------vehiculo n--------
for n = 1:size(vphist,3)
    for j = 1:size(vphist,1)
        xp(j+1,1,n)=xp(j,1,n)+T*vhist(n,j);
        for i = 1:(size(vphist,2)-1)
            xp(j,i+1,n)=xp(j,i,n)+T*vphist(j,i,n);
        end
    end
end

% % ---------vehiculo 2--------
% for j = 1:size(vp2hist,1)
%     xp_2(j+1,1)=xp_2(j,1)+T*vhist(2,j);   
%     for i = 1:(size(vp2hist,2)-1)
%         xp_2(j,i+1)=xp_2(j,i)+T*vp2hist(j,i);
%     end
% end
% 
% % ---------vehiculo 3--------
% for j = 1:size(vp3hist,1)
%     xp_3(j+1,1)=xp_3(j,1)+T*vhist(3,j);   
%     for i = 1:(size(vp3hist,2)-1)
%         xp_3(j,i+1)=xp_3(j,i)+T*vp3hist(j,i);
%     end
% 
% end




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
plot_car(an,alt,x(1,k),y(1),0,'b')%Plot the car
plot_car(an,alt,x(2,k),y(2),0,'r')%Plot the car
plot_car(an,alt,x(3,k),y(3),0,'g')%Plot the car
% 
% plot_car(an,alt,x_2(k),y_2(k),0,'r')%Plot the car
% plot_car(an,alt,x_3(k),y_3(k),0,'g')%Plot the car

hold on;
    
   


%-----------Plot trajectories------------    
%------------agente 1------------
    plot(x(1,k),y(k),'-r','linewidth',line_width);hold on % plot exhibited trajectory
    if k < size(vhist,2) % plot prediction
        plot(xp_1(k,1:N),zphist(k,1:N,1),'b--*')
    end
    plot(x_1(k)+xp,y_1(k)+yp,'--r')% plot robot circle
%------------agente 2------------
    plot(x_2(k),y_2(k),'-r','linewidth',line_width);hold on % plot exhibited trajectory
    if k < size(vhist,2) % plot prediction
        plot(xp_2(k,1:N),zp2hist(k,1:N),'r--*')
    end
    plot(x_2(k)+xp,y_2(k)+yp,'--r')% plot robot circle
%------------agente 3------------
    plot(x_3(k),y_3(k),'-r','linewidth',line_width);hold on % plot exhibited trajectory
    if k < size(vhist,2) % plot prediction
        plot(xp_3(k,1:N),zp3hist(k,1:N),'g--*')
    end
    plot(x_3(k)+xp,y_3(k)+yp,'--r')% plot robot circle
    
    
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