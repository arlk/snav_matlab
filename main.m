% rosshutdown; rosinit('http://192.168.1.1:11311');

clear all

% Time Step
dt = 0.01;

% TODO: Remove this section later
cmd_type_pub = rospublisher('/cmd_type', 'std_msgs/String');
cmd_type_msg = rosmessage(cmd_type_pub);

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
vel_sub = rossubscriber('/vel');
props_state_sub = rossubscriber('/props_state');
pause(2);

% TODO: Add to a struct file
ref.pos = [0 0 0]';
ref.vel = [0 0 0]';
ref.acc = [0 0 0]';
ref.yaw = 0;
ref.dyaw = 0;

% TODO: Remove Later
% cmd_type_msg.Data = 'SN_RC_RATES_CMD';
% send(cmd_type_pub, cmd_type_msg);

% Start Props
props_state.Data = true;
% props_state.Data = false;
while (~props_state.Data)
    props_state = receive(props_state_sub,1);
    send(gen_cmd_pub, gen_cmd_msg);
    send(start_props_pub, start_props_msg);
    pause(dt)
end

% Set Origin
[origin.pos, origin.ang] = unpackPoseMsg(receive(pose_sub,1));

tic; quad = MissionState.OnGround
while (1)
    t = toc;
    [pose.pos, pose.ang] = unpackPoseMsg(receive(pose_sub,1));
    [pose.vel, pose.omg] = unpackVelMsg(receive(vel_sub,1));
    if (quad == MissionState.OnGround)
        [ref, done] = takeoff(ref, pose, origin, t, dt);
        if (done)
            %ref.pos = pose.pos;
            tic; quad = MissionState.MissionReady
        end
    elseif (quad == MissionState.MissionReady)
        [ref, done] = mission(ref, pose, origin, t, dt);
        if (done)
            ref.pos = pose.pos;
            tic; quad = MissionState.MissionComplete
        end
    elseif (quad == MissionState.MissionComplete)
        [ref, done] = landing(ref, pose, origin, t, dt);
        if (done)
            break;
        end
    end
    traj_cmd_msg.Data = [ref.pos' ref.vel' ref.acc' ref.yaw ref.dyaw];
    send(traj_cmd_pub, traj_cmd_msg);
    [f, rate] = control(ref, pose, origin, t, dt)
    gen_cmd_msg.Linear.X = rate(2);
    gen_cmd_msg.Linear.Y = -rate(1);
    gen_cmd_msg.Linear.Z = f;
    gen_cmd_msg.Angular.Z = rate(3);
    % send(gen_cmd_pub, gen_cmd_msg);
    pause(dt);
end

send(stop_props_pub, stop_props_msg);
