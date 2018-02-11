function [ref, done] = landing(ref, pose, origin, t, dt)
landing_speed = 0.5;
ref.pos(3) = ref.pos(3) - landing_speed*dt;
done = (abs(pose.pos(3) - origin.pos(3)) < 0.01 || t > 5);
end
