% rosshutdown; rosinit('http://192.168.1.1:11311');

clear all

% Params
dt = 0.01;
takeoff_height = 2;
landing_speed = 0.5;

% Setup
start_props_pub = rospublisher('/start_props', 'std_msgs/Empty');
start_props_msg = rosmessage(start_props_pub);
stop_props_pub = rospublisher('/stop_props', 'std_msgs/Empty');
stop_props_msg = rosmessage(stop_props_pub);
gen_cmd_pub = rospublisher('/gen_cmd', 'geometry_msgs/Twist');
gen_cmd_msg = rosmessage(gen_cmd_pub);
traj_cmd_pub = rospublisher('/traj_cmd', 'std_msgs/Float32MultiArray');
traj_cmd_msg = rosmessage(traj_cmd_pub);
pose_sub = rossubscriber('/pose');
props_state_sub = rossubscriber('/props_state');
pause(2);

% Start Props
props_state.Data = false;
while (~props_state.Data)
    props_state = receive(props_state_sub,1);
    send(gen_cmd_pub, gen_cmd_msg);
    send(start_props_pub, start_props_msg);
    pause(dt)
end

% Set Origin
origin = getPose(receive(pose_sub,1));

tic; quad = MissionState.OnGround
while (1)
    t = toc;
    pose = getPose(receive(pose_sub,1));
    if (quad == MissionState.OnGround)
        pos_d = [0 0 takeoff_height] + origin.pos;
        if (takeoffcond(pose, origin, t))
            tic; quad = MissionState.MissionReady
        end
    elseif (quad == MissionState.MissionReady)
        mission_complete = mission(pose, origin, t)
        if (mission_complete)
            pos_d = pose.pos;
            tic; quad = MissionState.MissionComplete
        end
    elseif (quad == MissionState.MissionComplete)
        pos_d(3) = pos_d(3) - landing_speed*dt;
        if (landingcond(pose, origin, t))
            break;
        end
    end
    traj_cmd_msg.Data = [pos_d vel_d acc_d yaw_d yawd_d];
    send(traj_cmd_pub, traj_cmd_msg);
    pause(dt);
end

send(stop_props_pub, stop_props_msg);
