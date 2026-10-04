function J = FossenCostFunction(X, U, e, data, M, MA, DQ, DL, B, mass, buoyancy, com, cob, I, flipCost) 

    %constants
    weight = flipCost; %need to tune

    %pull thrusts
    tau = getForces(u, model.B);
    prev_tau = data.lastMV;
    
    %cost function
    sign_prev = sign(prev_tau);
    J = weight * max(0, (-sign_prev * tau))^2;

    %cost increases if signs are opposite
    %cost zero if equal sign
    %penalizes reversal from higher thrusts more
    %doesnt penalize dropping to zero (I think what we want)
end
