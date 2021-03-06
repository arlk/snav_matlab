function [pos, rot] = unpackPoseMsg(msg)
pos = [msg.Pose.Position.X, ...
    msg.Pose.Position.Y, ...
    msg.Pose.Position.Z]';
rot = quat2rotm([msg.Pose.Orientation.W, ...
    msg.Pose.Orientation.X, ...
    msg.Pose.Orientation.Y, ...
    msg.Pose.Orientation.Z]); % pitch roll yaw
end

