function fitDrag(forces, velocity, dof)
   idealMaxVelocity = max(velocity);
    
    %define A matrix
    A = zeros(length(velocity), 3);
    for m = 1:length(velocity)
        A(m, :) = [1, velocity(m), exp(velocity(m) / idealMaxVelocity)];
    end
    
    b = inv(transpose(A) * A) * transpose(A) * transpose(forces);
    
    tVals = linspace(0, idealMaxVelocity, 1000);
    curveVals = b(1) + tVals * b(2) + exp(tVals/idealMaxVelocity) * b(3);
    
    plot(velocity, forces, "ok", tVals, curveVals,"b")
    title(dof)
    xlabel("Velocity")
    ylabel("Force (N)")
    
    disp(dof)
    disp([b(1), b(2), b(3), idealMaxVelocity])
end