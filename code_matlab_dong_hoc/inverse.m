%% Kinemaitc simulation of mobile robot
clear all;clc; % lenh xoa man hinh
%% thong so vat ly cua xe
r = 0.03; % ban kinh banh xe
l_x = 0.05; % khoang cach banh xe den khung xe
l_y = 0.085;
%% vi tri ban dau
x_0 = 1;
y_0 = 0;
psi0 = pi/2;
%% vi tri cuoi
x_s = 10;
psi_s = pi/4;

viTri0 = [x_0;y_0;psi0]; % ma tran vi tri ban dau
viTri(:,1) = viTri0;

%% tham so mo phong
dt = 0.1; % buoc nhay
ts = 10; % thoi gian mo phong
t = 0:dt:ts; % vector

omega(:,1)=[0;0;0;0];
%% vong lap - xe di chuyen
for i=2:length(t)
    % phuong trinh quy dao
    x = x_0 + 0.15*i*dt;
    y = y_0 + 0.15*i*dt;
    q = psi0;
    viTri(:,i) = [x;y;q];
    psi = viTri(3,i);
    % ma tran jacobian
    J_psi = [cos(psi),sin(psi),0;
            -sin(psi),cos(psi),0;
             0,0,1];
    %% ma tran cau hinh xe
    W = 1/r*[ 1, -1, -(l_x+l_y);
              1,  1,  (l_x+l_y);
              1,  1, -(l_x+l_y);
              1, -1,  (l_x+l_y)];
    % van toc
    v_x = (x-viTri(1,i-1))/dt;
    v_y = (y-viTri(2,i-1))/dt;
    v_q = (q-viTri(3,i-1))/dt;
    vanToc(:,i) = [v_x,v_y,v_q];
    
    omega(:,i) = W*J_psi*vanToc(:,i);
    
    
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
ylabel('\eta,[units]');

omega(:,i)
%% hinh anh chuyen dong cua robot
l = 2*(l_y);
w = 2*l_x+0.02;
% toa do 4 diem robot so voi tam robot
mr_co = [-l/2,  l/2, l/2, -l/2, -l/2;           % |-----|
         -w/2, -w/2, w/2,  w/2, -w/2;];         % |_____|

figure
for i = 1:5:length(t) % robot bat dau di chuyen
    psi = viTri(3,i);
    R_psi = [cos(psi),-sin(psi);
           sin(psi),cos(psi);]; % ma tran quay
    v_pos = R_psi*mr_co;
    fill(v_pos(1,:) + viTri(1,i),v_pos(2,:)+viTri(2,i),'g')
    % backgound
    hold on, grid on
    axis([-1 5 -1 5 ]), axis square
    plot(viTri(1,1:i),viTri(2,1:i),'b-');
    legend('MR','Path')
    set(gca,'fontsize',8)
    xlabel('x,[m]');ylabel('y,[m]');
    pause(0.1);
    hold off
end % animation ends here


         