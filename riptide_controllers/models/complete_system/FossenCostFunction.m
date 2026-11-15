function J = FossenCostFunction(X, U, e, data, M, MA, DQ, DL, B, mass, buoyancy, com, cob, I, flipCost) 
%#codegen

    % Constants
    weight = flipCost; 
    deadband = 0.75; % Newtons deadband
    PH = size(U, 1) - 1;

    % Pull and align thrusts
    prev_real_thrust = reshape(data.LastMV, 8, 1); 
    prev_predicted_thrusts = U(1:PH, :); 
    prev_thrusts = [prev_real_thrust'; prev_predicted_thrusts]; 

    % Cost function: penalize sign reversals, allow dropping to zero
    sign_prev = tanh(prev_thrusts ./ deadband); 
    J_arr = weight .* max(0, (-sign_prev .* U)).^2; 
    J = sum(J_arr, "all"); 

end