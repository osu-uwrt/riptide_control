function yaw = quat_to_yaw(quat_bus)
    quat = [quat_bus.w, quat_bus.x, quat_bus.y, quat_bus.z];
    rotm = quat2rotm(quat);
    
    unit = [1, 0, 0];
    rotated = unit * rotm;
    yaw = -atan2(rotated(2), rotated(1));
end
