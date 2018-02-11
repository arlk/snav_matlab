function [ref, done] = takeoff(ref, pose, origin, t)
takeoff_height = 2;
ref.pos = [0 0 takeoff_height] + origin.pos;
done = (t > 2);
end