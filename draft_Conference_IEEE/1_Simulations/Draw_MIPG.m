function Draw_MIPG(vhist,vphist,zhist,zphist,dhist,T,N)

set(0,'DefaultAxesFontName', 'Times New Roman')
set(0,'DefaultAxesFontSize', 12)

line_width = 1.5;
fontsize_labels = 14;
% N
%--------------------------------------------------------------------------
%-----------------------Simulate robots -----------------------------------
%--------------------------------------------------------------------------
%-----agente 1----
x_r_1 = [];
y_r_1 = [];
%-----agente 2----
x_r_2 = [];
y_r_2 = [];
an = 5; alt=0.5; % parametros carroceria

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

xp_1=[T*vphist(1,:)];
xp_2=dhist(1:(size(vphist,2)));
zph_2=ones(81,size(vphist,2))*zhist(2);
for k = 1:size(vphist,1)
xp_1(k+1,:)=xp_1(k,:)+T*vphist(k,:);
xp_2(k+1,:)=xp_2(k,:)+T*15;
end
% filename = '2c_1obs.gif';%<---------------------------------------------------------------------------------------------------------------
x_1=[0];
x_2=[dhist(1)];

for k = 1:size(vhist,2)
   
    
    x_1(k+1)=x_1(k)+T*vhist(1,k);
    x_2(k+1)=x_2(k)+T*vhist(2,k);
    y_1(k)=zhist(1,k);
    y_2(k)=zhist(2,k);
%-------------Plot any car----------------    
plot_car(an,alt,x_1(k),y_1(k),0,'b')%Plot the car
plot_car(an,alt,x_2(k),y_2(k),0,'r')%Plot the car
hold on;
    
   
%-----------Plot trajectories------------    
%------------agente 1------------
    plot(x_1(k),y_1(k),'-r','linewidth',line_width);hold on % plot exhibited trajectory
    if k < size(vhist,2) % plot prediction
        plot(xp_1(k,1:N),zphist(k,1:N),'b--*')
    end
    plot(x_1(k)+xp,y_1(k)+yp,'--r')% plot robot circle
%------------agente 2------------
    plot(x_2(k),y_2(k),'-r','linewidth',line_width);hold on % plot exhibited trajectory
    if k < size(vhist,2) % plot prediction
        plot(xp_2(k,1:N),zph_2(k,1:N),'r--*')
    end
    plot(x_2(k)+xp,y_2(k)+yp,'--r')% plot robot circle
 
    
    
   hold off
    %figure(500)
    ylabel('$y$-position (m)','interpreter','latex','FontSize',fontsize_labels)
    xlabel('$x$-position (m)','interpreter','latex','FontSize',fontsize_labels)
    axis([-0.2 200 -0.2 6])%Axis description
    pause(0.1)
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