function mission(pose, t)
pose.pos = [msg.Pose.Position.X, ...
    msg.Pose.Position.Y, ...
    msg.Pose.Position.Z];
pose.ang = quat2eul([msg.Pose.Orientation.X, ...
    msg.Pose.Orientation.Y, ...
    msg.Pose.Orientation.Z, ...
    msg.Pose.Orientation.W]); % pitch roll yaw
end

