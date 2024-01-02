function axisError = angularError(bodyInWF, setInWF)
    %I Hate Quaternions and I Really hate them in matlab
    
    %Talos Heading in Body Frame
    Tbody = [1,0,0,0];
    
    %Transform from World to Body
    W2B = eul2quat(bodyInWF, "ZYX");
    
    %Transform from Body 2 World
    B2W = [W2B(1), -W2B(2), -W2B(3), -W2B(4)] / (norm(W2B) ^ 2);
    
    %Talos Heading in World Frame
    TWorld = quatmultiply(Tbody, W2B);
    
    %Talos Heading in World Frame Conjugate
    TWorldI = [TWorld(1), -TWorld(2), -TWorld(3), -TWorld(4)];
    
    %Set Point - In World Frame
    setQuat = eul2quat(setInWF, "ZYX");
    
    %Current Yaw Error
    eQuat = quatmultiply(setQuat, TWorldI);
    
    disp(eQuat)

    %axis Error
    axisError = [eQuat(4), eQuat(3), eQuat(2)];
    if(eQuat(1) < 0)
        axisError = -axisError;
    end

end

