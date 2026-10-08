function J = FossenCostFunction(X, U, e, data, M, MA, DQ, DL, B, mass, buoyancy, com, cob, I, flipCost) 

    %constants
    weight = flipCost; %need to tune
    deadband = 0.75; %in newtons
    PH = size(U,1) - 1;

    %pull thrusts
    prev_real_thrust = data.LastMV; % 8 by 1
    prev_predicted_thrusts = U(1:PH,:); % ph by 8
    prev_thrusts = [prev_real_thrust'; prev_predicted_thrusts]; % ph + 1 by 8
    %U 31 by 8

    %cost function
    sign_prev = tanh(prev_thrusts ./ deadband); % ph + 1 by 8 | smooths sign flipping
    J_arr = weight .* max(0, (-sign_prev .* U)).^2; % ph + 1 by 8
    J = sum(J_arr, "all"); % 1 by 1

    %cost increases if signs are opposite
    %cost zero if equal sign
    %penalizes reversal from higher thrusts more
    %doesnt penalize dropping to zero (I think what we want)
end
