%% Kinemaitc simulation of mobile robot
clear all;clc; % lenh xoa man hinh

%% tham so mo phong
dt = 0.1; % buoc nhay
ts = 30; % thoi gian mo phong
t = 0:dt:ts; % vector

%% thong so vat ly cua xe
r = 0.03; % ban kinh banh xe
l_x = 0.05; % khoang cach banh xe den khung xe
l_y = 0.085;

%% vi tri ban dau
x0 = 0;
y0 = 2;
psi0 = pi/4;

eta0 = [x0;y0;psi0]; % ma tran vi tri ban dau
eta(:,1) = eta0;

%% vong lap - xe di chuyen
for i=1:length(t)
    
    %% input
    % quy dao mong muon
    eta_d(:,i) = [7*sin(0.1*t(i));4-4*cos(0.5*t(i));pi/2];
    % dao ham quy dao mong muon
    eta_d_dot = [0.7*cos(0.1*t(i));2.5*sin(0.5*t(i));0];
    % tinh sai so vi tri
    eta_error = eta_d(:,i) - eta(:,i);

    psi = eta(3,i);
    % ma tran chuyen doi
    J = [cos(psi),-sin(psi),0;
             sin(psi),cos(psi),0;
             0,0,1];
    
    W = r/4*[1,1,1,1;
            -1,1,1,-1;
            -1/(l_x+l_y),1/(l_x+l_y),-1/(l_x+l_y),1/(l_x+l_y)];
   
    W_1 = 1/r*[ 1, -1, -(l_x+l_y);
              1,  1,  (l_x+l_y);
              1,  1, -(l_x+l_y);
              1, -1,  (l_x+l_y)];
  
    % tinh omega
    omega(:,i) = W_1*(inv(J)*(eta_d_dot + eta_error));
    % van toc trong XGY
    vtXY(:,i) = J*(W*omega(:,i));
    eta(:,i+1) = eta(:,i) + vtXY(:,i)*dt; % update vi tri
    
end
%% ve bieu do
figure
plot(t,omega(1,1:i),'r-'); % toa do Ox
hold on
plot(t, omega(2,1:i),'b-'); % toa do Oy
plot(t, omega(3,1:i),'m-'); % goc phi
plot(t, omega(4,1:i),'g-');
legend('w1,[rad/s]','w2,[rad/s]','w3,[rad/s]','w4,rad/s');
set(gca,'fontsize',10)
xlabel('t,[s]');
ylabel('?,[rad/s]');

eta(:,i)
%% hinh anh chuyen dong cua robot
w = 2*(l_x+0.02);
l = 2*l_y;
% toa do 4 diem robot so voi tam robot
mr_co = [-l/2,  l/2, l/2, -l/2, -l/2;           % |-----|
         -w/2, -w/2, w/2,  w/2, -w/2;];         % |_____|

figure
for i = 1:5:length(t) % robot bat dau di chuyen
    psi = eta(3,i);
    R_psi = [cos(psi),-sin(psi);
           sin(psi),cos(psi);]; % ma tran quay
    v_pos = R_psi*mr_co;
    fill(v_pos(1,:) + eta(1,i),v_pos(2,:)+eta(2,i),'g')
    % backgound
    hold on, grid on
    plot(eta_d(1,:),eta_d(2,:),'r--');
    axis([-1 10 -1 10 ]), axis square
    plot(eta(1,1:i),eta(2,1:i),'b-');
    legend('MR','Desired','Path')
    set(gca,'fontsize',8)
    xlabel('x,[m]');ylabel('y,[m]');
    pause(0.1);
    hold off
end % animation ends here


         