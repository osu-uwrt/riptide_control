clear; 
%radial curve fitting


%generate random curve
period = 1.3;
tVals = linspace(0, period, 1000);
yVals = sin(2*pi() * tVals) + tVals.^2*20 + 20*cos(4* pi() *tVals) + exp(tVals) * 3; 

plot(tVals, yVals)    



%setup curve fitting
%should be odd
functionCount = 21;
offsets = linspace(-pi(), pi() - 2*pi()/functionCount, functionCount);

b = fitCurve(tVals, yVals, functionCount);

% curveVals = zeros(1, length(tVals));
% for i=1:functionCount
%     curveVals = curveVals + b(i) * (sin(pi() .* tVals ./ (2*period) + offsets(i))).^functionCount;
% end

for val = 1:length(tVals)
    curveVals(val) = evalCurve(b, tVals(val) / period);
end

figure(1)
plot(tVals, yVals, tVals, curveVals)

figure(2)
plot(linspace(0, period, functionCount), b)
