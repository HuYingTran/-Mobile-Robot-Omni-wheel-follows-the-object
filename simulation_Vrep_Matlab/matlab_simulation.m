clc;clear all;

sim=remApi('remoteApi');
sim.simxFinish(-1);
clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5);

%% tham so mo phong
dt = 0.1; % buoc nhay
ts = 5; % thoi gian mo phong
t = 0:dt:ts; % vector thoi gian

%% thong so vat ly cua xe
r = 0.1/2.0%0.0475; % ban kinh banh xe
l_x = 0.471/2.0; % khoang cach banh xe den khung xe
l_y = 0.30/2.0;

%% vi tri ban dau
x0 = -2;
y0 = -2;
psi0 = pi/2;
eta0 = [x0;y0;psi0]; % ma tran vi tri ban dau
eta(:,1) = eta0;

%% ma tran dong hoc
% thuan
W = r/4*[          1,           1,            1,          1;
                  -1,           1,            1,         -1;
        -1/(l_x+l_y), 1/(l_x+l_y), -1/(l_x+l_y), 1/(l_x+l_y)];
% nghich ----------------------------------------------------  
W_1 = 1/r*[ 1, -1, -(l_x+l_y);
            1,  1,  (l_x+l_y);
            1,  1, -(l_x+l_y);
            1, -1,  (l_x+l_y)];
 
%% vong lap tinh toan      
for i=1:length(t)

    % quy dao mong muon
    eta_d(:,i) = [-2;-2+0.4*t(i);pi/2];
    % dao ham quy dao mong muon
    eta_d_dot = [0;0.4;0];
    % tinh sai so vi tri
    eta_error = eta_d(:,i) - eta(:,i);

    psi = eta(3,i);
    % ma tran chuyen doi
    J = [cos(psi),-sin(psi),0;
             sin(psi),cos(psi),0;
             0,0,1];
    
    % tinh omega
    omega(:,i) = W_1*(inv(J)*(eta_d_dot + eta_error));
    % van toc trong XGY
    vtXY(:,i) = J*(W*omega(:,i));
    
    eta(:,i+1) = eta(:,i) + vtXY(:,i)*dt; % update vi tri thuc

end


%% hinh anh chuyen dong cua robot
% cau hinh xe
l = 2*(l_x+0.02);
w = 2*l_y;
% toa do 4 diem robot so voi tam robot
mr_co = [-l/2,  l/2, l/2, -l/2, -l/2;           % |-----|
         -w/2, -w/2, w/2,  w/2, -w/2;];         % |_____|
for i=1:length(t)
    psi = eta(3,i);
    R_psi = [cos(psi),-sin(psi);
             sin(psi),cos(psi);]; % ma tran quay
    v_pos = R_psi*mr_co;
end    
     
  

if (clientID>-1)
    disp('connected')
    figure
    for i=1:length(t)
        fill(v_pos(1,:) + eta(1,i),v_pos(2,:)+eta(2,i),'g')
        hold on, grid on
        plot(eta_d(1,:),eta_d(2,:),'r--');
        axis([-3 3 -3 3]), axis square
        plot(eta(1,1:i),eta(2,1:i),'b-');
        set(gca,'fontsize',8)
        xlabel('x,[m]');ylabel('y,[m]');
        %pause(0.01);
        hold off
        
        [returnCode,left_Motor3]=sim.simxGetObjectHandle(clientID,'rollingJoint_rl',sim.simx_opmode_blocking);
        [returnCode]=sim.simxSetJointTargetVelocity(clientID,left_Motor3,omega(3,i),sim.simx_opmode_blocking);
    
        [returnCode,left_Motor4]=sim.simxGetObjectHandle(clientID,'rollingJoint_rr',sim.simx_opmode_blocking);
        [returnCode]=sim.simxSetJointTargetVelocity(clientID,left_Motor4,omega(4,i),sim.simx_opmode_blocking);

        [returnCode,left_Motor2]=sim.simxGetObjectHandle(clientID,'rollingJoint_fr',sim.simx_opmode_blocking);
        [returnCode]=sim.simxSetJointTargetVelocity(clientID,left_Motor2,omega(2,i),sim.simx_opmode_blocking);
        
        [returnCode,left_Motor1]=sim.simxGetObjectHandle(clientID,'rollingJoint_fl',sim.simx_opmode_blocking);
        [returnCode]=sim.simxSetJointTargetVelocity(clientID,left_Motor1,omega(1,i),sim.simx_opmode_blocking);

    end
    legend('MR','Desired','Path')
    
    [returnCode]=sim.simxSetJointTargetVelocity(clientID,left_Motor1,0,sim.simx_opmode_blocking);
    [returnCode]=sim.simxSetJointTargetVelocity(clientID,left_Motor2,0,sim.simx_opmode_blocking);
    [returnCode]=sim.simxSetJointTargetVelocity(clientID,left_Motor3,0,sim.simx_opmode_blocking);
    [returnCode]=sim.simxSetJointTargetVelocity(clientID,left_Motor4,0,sim.simx_opmode_blocking); 
    sim.simxFinish(-1);
end
sim.delete();

pause(1)
%% ve bieu do omega
figure
plot(t,omega(1,1:i),'r-');
hold on
plot(t, omega(2,1:i),'b-');
plot(t, omega(3,1:i),'m-');
plot(t, omega(4,1:i),'g-');
legend('w1,[rad/s]','w2,[rad/s]','w3,[rad/s]','w4,rad/s');
set(gca,'fontsize',10)
xlabel('t,[s]');
ylabel('?,[rad/s]');
% -----------------------------------------------------------

%% ve bieu do vi tri (x,y,q)
figure
plot(t,eta(1,1:i),'r-'); % toa do Ox
hold on
plot(t, eta(2,1:i),'b-'); % toa do Oy
plot(t, eta(3,1:i),'g-'); % goc phi
legend('x,[m]','y,[m]','\psi,[rad]');
set(gca,'fontsize',10)
xlabel('t,[s]');
ylabel('\eta,[units]');
% -----------------------------------------------------------