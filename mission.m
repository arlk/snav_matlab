function [ref, done] = mission(ref, pose, origin, t, dt)
ref.pos(1) = origin.pos(1);
ref.pos(2) = origin.pos(2);
done = (t > 15);
end
