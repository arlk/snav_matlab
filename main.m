% rosshutdown; rosinit('http://192.168.1.1:11311');

clear all

% Time Step
dt = 0.01;

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

% TODO: Add to a struct file
ref.pos = [0 0 0];
ref.vel = [0 0 0];
ref.acc = [0 0 0];
ref.yaw = 0;
ref.dyaw = 0;

% Start Props
props_state.Data = false;
while (~props_state.Data)
    props_state = receive(props_state_sub,1);
    send(gen_cmd_pub, gen_cmd_msg);
    send(start_props_pub, start_props_msg);
    pause(dt)
end

% Set Origin
origin = unpackPoseMsg(receive(pose_sub,1));

tic; quad = MissionState.OnGround
while (1)
    t = toc;
    pose = unpackPoseMsg(receive(pose_sub,1));
    if (quad == MissionState.OnGround)
        [ref, done] = takeoff(ref, pose, origin, t);
        if (done)
            ref.pos = pose.pos;
            tic; quad = MissionState.MissionReady
        end
    elseif (quad == MissionState.MissionReady)
        [ref, done] = mission(ref, pose, origin, t);
        if (done)
            ref.pos = pose.pos;
            tic; quad = MissionState.MissionComplete
        end
    elseif (quad == MissionState.MissionComplete)
        [ref, done] = landing(ref, pose, origin, t);
        if (done)
            break;
        end
    end
    traj_cmd_msg.Data = [ref.pos ref.vel ref.acc ref.yaw ref.dyaw];
    send(traj_cmd_pub, traj_cmd_msg);
    pause(dt);
end

send(stop_props_pub, stop_props_msg);
