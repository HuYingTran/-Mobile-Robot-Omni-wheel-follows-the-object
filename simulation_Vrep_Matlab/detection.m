clc;clear all;

sim=remApi('remoteApi');
sim.simxFinish(-1);
clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5);

t = 100000;

if (clientID>-1)
    disp('connected')
    
        [returnCode,left_Motor1]=sim.simxGetObjectHandle(clientID,'rollingJoint_rr',sim.simx_opmode_blocking);
        [returnCode,left_Motor2]=sim.simxGetObjectHandle(clientID,'rollingJoint_rl',sim.simx_opmode_blocking);
        [returnCode,left_Motor3]=sim.simxGetObjectHandle(clientID,'rollingJoint_fr',sim.simx_opmode_blocking);
        [returnCode,left_Motor4]=sim.simxGetObjectHandle(clientID,'rollingJoint_fl',sim.simx_opmode_blocking);
        [returnCode,camera]=sim.simxGetObjectHandle(clientID,'Vision_sensor',sim.simx_opmode_blocking);
        [returnCode,resolution,image]=sim.simxGetVisionSensorImage2(clientID,camera,0,sim.simx_opmode_streaming);
        pause(1);
    for i=1:t
        [returnCode,resolution,image]=sim.simxGetVisionSensorImage2(clientID,camera,0,sim.simx_opmode_buffer);
        % xu ly anh
        % -------------------------------------------------------------------------------------------------
        data = image;
        diff_im = imsubtract(data(:,:,1), rgb2gray(data));
        %Use a median filter to filter out noise
        diff_im = medfilt2(diff_im, [3 3]);
        diff_im = im2bw(diff_im,0.18);
        diff_im = bwareaopen(diff_im,300);
        bw = bwlabel(diff_im, 8);
        stats = regionprops(bw, 'BoundingBox', 'Centroid');
        % Display the image
        for object = 1:length(stats)
            bb = stats(object).BoundingBox; % vi tri 2 diem ve box
            bc = stats(object).Centroid;    % toa do tam
            rectangle('Position',bb,'EdgeColor','r','LineWidth',2);
        end
        % ------------------------------------------------------------------------------------------
        if (bb(1)+bb(3)>=400)
            [returnCode]=sim.simxSetJointTargetVelocity(clientID,left_Motor3,0,sim.simx_opmode_blocking);
            [returnCode]=sim.simxSetJointTargetVelocity(clientID,left_Motor4,0,sim.simx_opmode_blocking);
            [returnCode]=sim.simxSetJointTargetVelocity(clientID,left_Motor2,0,sim.simx_opmode_blocking);
            [returnCode]=sim.simxSetJointTargetVelocity(clientID,left_Motor1,0,sim.simx_opmode_blocking);
        else
            [returnCode]=sim.simxSetJointTargetVelocity(clientID,left_Motor3,1,sim.simx_opmode_blocking);
            [returnCode]=sim.simxSetJointTargetVelocity(clientID,left_Motor4,1,sim.simx_opmode_blocking);
            [returnCode]=sim.simxSetJointTargetVelocity(clientID,left_Motor2,1,sim.simx_opmode_blocking);
            [returnCode]=sim.simxSetJointTargetVelocity(clientID,left_Motor1,1,sim.simx_opmode_blocking);
        end
        
        if bc(1)>= 260
            [returnCode]=sim.simxSetJointTargetVelocity(clientID,left_Motor1,1,sim.simx_opmode_blocking);
            [returnCode]=sim.simxSetJointTargetVelocity(clientID,left_Motor2,-1,sim.simx_opmode_blocking);
            [returnCode]=sim.simxSetJointTargetVelocity(clientID,left_Motor3,-1,sim.simx_opmode_blocking);
            [returnCode]=sim.simxSetJointTargetVelocity(clientID,left_Motor4,1,sim.simx_opmode_blocking);
        end
        if bc(1)<=250
            [returnCode]=sim.simxSetJointTargetVelocity(clientID,left_Motor1,-1,sim.simx_opmode_blocking);
            [returnCode]=sim.simxSetJointTargetVelocity(clientID,left_Motor2,1,sim.simx_opmode_blocking);
            [returnCode]=sim.simxSetJointTargetVelocity(clientID,left_Motor3,1,sim.simx_opmode_blocking);
            [returnCode]=sim.simxSetJointTargetVelocity(clientID,left_Motor4,-1,sim.simx_opmode_blocking);
        end
    end
    [returnCode]=sim.simxSetJointTargetVelocity(clientID,left_Motor1,0,sim.simx_opmode_blocking);
    [returnCode]=sim.simxSetJointTargetVelocity(clientID,left_Motor2,0,sim.simx_opmode_blocking);
    [returnCode]=sim.simxSetJointTargetVelocity(clientID,left_Motor3,0,sim.simx_opmode_blocking);
    [returnCode]=sim.simxSetJointTargetVelocity(clientID,left_Motor4,0,sim.simx_opmode_blocking); 
    sim.simxFinish(-1);
end
sim.delete();
