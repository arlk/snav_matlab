function landing_complete = landingcond(pose, origin, t)
landing_complete = (abs(pose.pos(3) - origin.pos(3)) < 0.01 || t > 5)
end
