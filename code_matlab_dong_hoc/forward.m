%% Kinemaitc simulation of mobile robot
clear all;clc; % lenh xoa man hinh

%% tham so mo phong
dt = 0.1; % buoc nhay
ts = 10; % thoi gian mo phong
t = 0:dt:ts; % vector

%% thong so vat ly cua xe
r = 0.03; % ban kinh banh xe
l_x = 0.05; % khoang cach banh xe den khung xe
l_y = 0.085;

%% vi tri ban dau
x0 = 1;
y0 = 0;
psi0 = pi()/2;

eta0 = [x0;y0;psi0]; % ma tran vi tri ban dau
eta(:,1) = eta0;

%% vong lap - xe di chuyen
for i=1:length(t)
    
    %% input
    omega_1 = 10;
    omega_2 = 0;
    omega_3 = 0;
    omega_4 = 10;
    omega = [omega_1;omega_2;omega_3;omega_4];
    psi = eta(3,i);
    % ma tran jacobian
    J_psi = [cos(psi),-sin(psi),0;
             sin(psi),cos(psi),0;
             0,0,1];
    
    %% ma tran cau hinh xe
    W = r/4*[1,1,1,1;
            -1,1,1,-1;
            -1/(l_x+l_y),1/(l_x+l_y),-1/(l_x+l_y),1/(l_x+l_y)];
    % van toc
    zeta(:,i) = W*omega;
    
    % dao ham toa do
    eta_dot(:,i) = J_psi*zeta(:,i);
    
    %% ham truyen vi tri bang ohuong phap euler
    eta(:,i+1) = eta(:,i) + dt*eta_dot(:,i); % update vi tri
    
end
%% ve bieu do
figure
plot(t,eta(1,1:i),'r-'); % toa do Ox
hold on
plot(t, eta(2,1:i),'b-'); % toa do Oy
plot(t, eta(3,1:i),'g-'); % goc phi
legend('x,[m]','y,[m]','\psi,[rad]');
set(gca,'fontsize',10)
xlabel('t,[s]');
ylabel('\eta,[units]');

eta(:,i)
%% hinh anh chuyen dong cua robot
l = 2*(l_y);
w = 2*l_x+0.02;
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
    axis([-1 5 -1 5 ]), axis square
    plot(eta(1,1:i),eta(2,1:i),'b-');
    legend('MR','Path')
    set(gca,'fontsize',8)
    xlabel('x,[m]');ylabel('y,[m]');
    pause(0.1);
    hold off
end % animation ends here


         