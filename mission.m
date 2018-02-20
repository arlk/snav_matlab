function [ref, done] = mission(ref, pose, origin, t, dt)
ref.pos(1) = origin.pos(1) + 0.1;
ref.pos(2) = origin.pos(2) + 0.2;
done = (t > 5);
end
