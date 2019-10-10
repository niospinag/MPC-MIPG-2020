function plot_car(an,alt,x,y,theta,color)
 an2 = (an/2)*0.7; alt2=alt/2; % parametros distancia llantas
 an3 = an*0.2; alt3=alt*0.1; % parametros llantas

%-------------agente 1----------------    
%     x1 = x; y1 = y; th1 = theta;
    
    x1 = x; y1 = y; th1 = theta;
    x1_cu = [ x1-(an/2), x1-(an/2),x1+(an/2), x1+(an/2)];%,x1+(h_t/3)*cos(th1)];
    y1_cu = [ y1+(alt/2),y1-(alt/2), y1-(alt/2), y1+(alt/2)];%,y1+(h_t/3)*sin(th1)];
    fill(x1_cu, y1_cu, color); % plot reference state
    hold on;
    %--llanta 1
    x1_ll = [ x1-an2-(an3/2), x1-an2-(an3/2),x1-an2+(an3/2), x1-an2+(an3/2)];%,x1-an2+(h_t/3)*cos(th1)];
    y1_ll = [ y1+alt2+(alt3/2),y1+alt2-(alt3/2), y1+alt2-(alt3/2), y1+alt2+(alt3/2)];%,y1+alt2+(h_t/3)*sin(th1)];
    fill(x1_ll, y1_ll, 'k'); % plot reference state
    hold on;
       %--llanta 2
    x1_ll = [ x1-an2-(an3/2), x1-an2-(an3/2),x1-an2+(an3/2), x1-an2+(an3/2)];%,x1-an2+(h_t/3)*cos(th1)];
    y1_ll = [ y1-alt2+(alt3/2),y1-alt2-(alt3/2), y1-alt2-(alt3/2), y1-alt2+(alt3/2)];%,y1+alt2+(h_t/3)*sin(th1)];
    fill(x1_ll, y1_ll, 'k'); % plot reference state
    hold on;
       %--llanta 3
    x1_ll = [ x1+an2-(an3/2), x1+an2-(an3/2),x1+an2+(an3/2), x1+an2+(an3/2)];%,x1-an2+(h_t/3)*cos(th1)];
    y1_ll = [ y1+alt2+(alt3/2),y1+alt2-(alt3/2), y1+alt2-(alt3/2), y1+alt2+(alt3/2)];%,y1+alt2+(h_t/3)*sin(th1)];
    fill(x1_ll, y1_ll, 'k'); % plot reference state
    hold on;
       %--llanta 4
    x1_ll = [ x1+an2-(an3/2), x1+an2-(an3/2),x1+an2+(an3/2), x1+an2+(an3/2)];%,x1-an2+(h_t/3)*cos(th1)];
    y1_ll = [ y1-alt2+(alt3/2),y1-alt2-(alt3/2), y1-alt2-(alt3/2), y1-alt2+(alt3/2)];%,y1+alt2+(h_t/3)*sin(th1)];
    fill(x1_ll, y1_ll, 'k'); % plot reference state
    hold on;