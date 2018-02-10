% rosshutdown; rosinit('http://192.168.1.1:11311');

clear all
% Setup
start_props_pub = rospublisher('/start_props', 'std_msgs/Empty');
stop_props_pub = rospublisher('/stop_props', 'std_msgs/Empty');
cmd_vel_pub = rospublisher('/cmd_vel', 'geometry_msgs/Twist');
traj_cmd_pub = rospublisher('/traj_cmd', 'std_msgs/Float32MultiArray');
pose_sub = rossubscriber('/pose');
props_state_sub = rossubscriber('/props_state');
start_props_msg = rosmessage(start_props_pub);
stop_props_msg = rosmessage(stop_props_pub);
cmd_vel_msg = rosmessage(cmd_vel_pub);
traj_cmd_msg = rosmessage(traj_cmd_pub);
pause(2);

tmp_msg = receive(pose_sub,1);
origin.pos = [tmp_msg.Pose.Position.X, ...
    tmp_msg.Pose.Position.Y, ...
    tmp_msg.Pose.Position.Z];
origin.ang = quat2eul([tmp_msg.Pose.Orientation.X, ...
    tmp_msg.Pose.Orientation.Y, ...
    tmp_msg.Pose.Orientation.Z, ...
    tmp_msg.Pose.Orientation.W]); % pitch roll yaw

pos_d = [0 0 0] + origin.pos;
vel_d = [0 0 0];
acc_d = [0 0 0];
yaw_d = 0 + origin.ang(end);
yawd_d = 0;
traj_cmd_msg.Data = [pos_d vel_d acc_d yaw_d yawd_d];

props_state.Data = false;
while (~props_state.Data)
props_state = receive(props_state_sub,1);
send(start_props_pub, start_props_msg);
send(cmd_vel_pub, cmd_vel_msg);
end

dt = 0.01;
quad = MissionState.OnGround
tic;
while (1)
    t = toc;
    pose = getPose(receive(pose_sub,1));
    if (quad == MissionState.OnGround)
        takeoff_height = 2;
        pos_d = [0 0 takeoff_height] + origin.pos;
        if (t >= 2)
            quad = MissionState.MissionReady
            tic;
        end
    elseif (quad == MissionState.MissionReady)
        % do nothing
        if (t >= 5)
            quad = MissionState.MissionComplete
            pos_d = pose.pos;
            tic;
        end
    elseif (quad == MissionState.MissionComplete)
        landing_speed = 0.5;
        pos_d(3) = pos_d(3) - landing_speed*dt;
        if (abs(pose.pos(3) - origin.pos(3))<0.01 || t > 5)
            break;
        end
    end
    traj_cmd_msg.Data = [pos_d vel_d acc_d yaw_d yawd_d];
    send(traj_cmd_pub, traj_cmd_msg);
    pause(dt);
end

send(stop_props_pub, stop_props_msg);
