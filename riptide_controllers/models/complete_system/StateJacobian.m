function [A, Bmv] = StateJacobian(x, thrusterForces, M, MA, DQ, DL, B, mass, buoyancy, com, cob, I, flipCost) 

Bmv = [zeros(7,8); M\B];



