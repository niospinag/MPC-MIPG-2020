function raw_MPC_point_stabilization_v1 (t,xx_1,xx1_1,u_cl_1,xs_1,xx_2,xx1_2,u_cl_2,xs_2,N,rob_diam,obs_x,obs_y,obs_diam)
%Draw_MPC_point_stabilization_v1 (t,xx_1,xx1_1,u_cl_1,xs_1,xx_2,xx1_2,u_cl_2,xs_2,N,rob_diam)


set(0,'DefaultAxesFontName', 'Times New Roman')
set(0,'DefaultAxesFontSize', 12)

line_width = 1.5;
fontsize_labels = 14;

%--------------------------------------------------------------------------
%-----------------------Simulate robots -----------------------------------
%--------------------------------------------------------------------------
%-----agente 1----
x_r_1 = [];
y_r_1 = [];
%-----agente 2----
x_r_2 = [];
y_r_2 = [];


r = rob_diam/2;  % robot radius
ang=0:0.005:2*pi;
xp=r*cos(ang);
yp=r*sin(ang);

r = obs_diam/2;  % obstacle radius
xp_obs=r*cos(ang);
yp_obs=r*sin(ang);


figure(500)%create a window named "500"
% Animate the robot motion
%figure;%('Position',[200 200 1280 720]);
set(gcf,'PaperPositionMode','auto')
set(gcf, 'Color', 'w');
set(gcf,'Units','normalized','OuterPosition',[0 0 2 0.5]);%tamaño del grafico

filename = '2c_1obs.gif';%<---------------------------------------------------------------------------------------------------------------


for k = 1:size(xx_1,2)
    h_t = 0.14; w_t=0.09; % triangle parameters
%-------------agente 1----------------    
    x1 = xs_1(1); y1 = xs_1(2); th1 = xs_1(3);
    x1_tri = [ x1+h_t*cos(th1), x1+(w_t/2)*cos((pi/2)-th1), x1-(w_t/2)*cos((pi/2)-th1)];%,x1+(h_t/3)*cos(th1)];
    y1_tri = [ y1+h_t*sin(th1), y1-(w_t/2)*sin((pi/2)-th1), y1+(w_t/2)*sin((pi/2)-th1)];%,y1+(h_t/3)*sin(th1)];
    fill(x1_tri, y1_tri, 'g'); % plot reference state
    hold on;
    x1 = xx_1(1,k,1); y1 = xx_1(2,k,1); th1 = xx_1(3,k,1);
    x_r_1 = [x_r_1 x1];
    y_r_1 = [y_r_1 y1];
    x1_tri = [ x1+h_t*cos(th1), x1+(w_t/2)*cos((pi/2)-th1), x1-(w_t/2)*cos((pi/2)-th1)];%,x1+(h_t/3)*cos(th1)];
    y1_tri = [ y1+h_t*sin(th1), y1-(w_t/2)*sin((pi/2)-th1), y1+(w_t/2)*sin((pi/2)-th1)];%,y1+(h_t/3)*sin(th1)];
%-----------agente 2------------ 
    
    x2 = xs_2(1); y2 = xs_2(2); th2 = xs_2(3);
    x2_tri = [ x2+h_t*cos(th2), x2+(w_t/2)*cos((pi/2)-th2), x2-(w_t/2)*cos((pi/2)-th2)];%,x1+(h_t/3)*cos(th1)];
    y2_tri = [ y2+h_t*sin(th2), y2-(w_t/2)*sin((pi/2)-th2), y2+(w_t/2)*sin((pi/2)-th2)];%,y1+(h_t/3)*sin(th1)];
    fill(x2_tri, y2_tri, 'g'); % plot reference state
    hold on;
    x2 = xx_2(1,k,1); y2 = xx_2(2,k,1); th2 = xx_2(3,k,1);
    x_r_2 = [x_r_2 x2];
    y_r_2 = [y_r_2 y2];
    x2_tri = [ x2+h_t*cos(th2), x2+(w_t/2)*cos((pi/2)-th2), x2-(w_t/2)*cos((pi/2)-th2)];%,x1+(h_t/3)*cos(th1)];
    y2_tri = [ y2+h_t*sin(th2), y2-(w_t/2)*sin((pi/2)-th2), y2+(w_t/2)*sin((pi/2)-th2)];%,y1+(h_t/3)*sin(th1)];

    
    
    
%-----------agente 1------------    
    plot(x_r_1,y_r_1,'-r','linewidth',line_width);hold on % plot exhibited trajectory
    if k < size(xx_1,2) % plot prediction
        plot(xx1_1(1:N,1,k),xx1_1(1:N,2,k),'r--*')
    end
    
    fill(x1_tri, y1_tri, 'r'); % plot robot position
    plot(x1+xp,y1+yp,'--r'); % plot robot circle
%------------agente 2------------

    plot(x_r_2,y_r_2,'-r','linewidth',line_width);hold on % plot exhibited trajectory
    if k < size(xx_2,2) % plot prediction
        plot(xx1_2(1:N,1,k),xx1_2(1:N,2,k),'r--*')
    end
    
    fill(x2_tri, y2_tri, 'r'); % plot robot position
    plot(x2+xp,y2+yp,'--r'); % plot robot circle

    plot(obs_x+xp_obs,obs_y+yp_obs,'--b'); % plot obstacle circle    

   
    hold off
    %figure(500)
    ylabel('$y$-position (m)','interpreter','latex','FontSize',fontsize_labels)
    xlabel('$x$-position (m)','interpreter','latex','FontSize',fontsize_labels)
    axis([-0.2 7 -0.2 1.8])
    pause(0.1)
    box on;
    grid on
    %aviobj = addframe(aviobj,gcf);
    drawnow
    % for video generation
    F(k) = getframe(gcf); % to get the current frame
    
    
    %---------------------make gift--------------------------
frame = getframe(figure(500)); 
      im = frame2im(frame); 
      [imind,cm] = rgb2ind(im,256); 
      % Write to the GIF File 
      if k == 1 
          imwrite(imind,cm,filename,'gif', 'Loopcount',inf); 
      else 
          imwrite(imind,cm,filename,'gif','WriteMode','append'); 
      end 

    
    
end
close(gcf)

%------------------------make video----------------------
% viobj = close(aviobj)
video = VideoWriter('2c_obs.avi','Uncompressed AVI');%<-----------------------------------------------------------------------------------

video = VideoWriter('2c_obs.avi','Motion JPEG AVI');%<--------------------------------------------------------------------------------
video.FrameRate = 5;  % (frames per second) this number depends on the sampling time and the number of frames you have
open(video)
writeVideo(video,F)
close (video)

%-----------------------Plot controls history----------------------------------------------------
%------------------------agent 1-------------------
figure
subplot(221)
stairs(t,u_cl_1(:,1),'k','linewidth',2); axis([0 t(end) -0.35 0.75])
xlabel('time (seconds)')
ylabel('v1 (rad/s)')
grid on
subplot(222)
stairs(t,u_cl_1(:,2),'r','linewidth',2); axis([0 t(end) -0.85 0.85])
xlabel('time (seconds)')
ylabel('\omega 1 (rad/s)')
grid on

%------------------------agent 2-------------------

subplot(223)
stairs(t,u_cl_2(:,1),'m','linewidth',2); axis([0 t(end) -0.35 0.75])
xlabel('time (seconds)')
ylabel('v2 (rad/s)')
grid on
subplot(224)
stairs(t,u_cl_2(:,2),'b','linewidth',2); axis([0 t(end) -0.85 0.85])
xlabel('time (seconds)')
ylabel('\omega 2 (rad/s)')
grid on



%-----------------------Plot states history----------------------------------------------------
%------------------------agent 1-------------------
figure
subplot(231)% position x
plot(t,xx_1(1,1:64),'k','linewidth',2);% axis([0 t(end) -4 4])
xlabel('time (seconds)')
ylabel('x (m)')
grid on
subplot(232)% position y
plot(t,xx_1(2,1:64),'r','linewidth',2);% axis([0 t(end) -4 4])
xlabel('time (seconds)')
ylabel('y (m)')
grid on
subplot(233)% position theta
plot(t,xx_1(3,1:64),'b','linewidth',2);% axis([0 t(end) -4 4])
xlabel('time (seconds)')
ylabel('\theta (rad/s)')
grid on
%------------------------agent 2-------------------
subplot(234)% position x
plot(t,xx_2(1,1:64),'k','linewidth',2);% axis([0 t(end) -4 4])
xlabel('time (seconds)')
ylabel('x (m)')
grid on
subplot(235)% position y
plot(t,xx_2(2,1:64),'r','linewidth',2);% axis([0 t(end) -4 4])
xlabel('time (seconds)')
ylabel('y (m)')
grid on
subplot(236)% position theta
plot(t,xx_2(3,1:64),'b','linewidth',2);% axis([0 t(end) -4 4])
xlabel('time (seconds)')
ylabel('\theta (rad/s)')
grid on
