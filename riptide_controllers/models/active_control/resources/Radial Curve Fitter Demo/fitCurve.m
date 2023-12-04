function co = fitCurve(tVals, yVals, functionCount)
    %tVals - x-axis values - in case of motion profiles time
    %yVals - y-aixs values
    %functionCount - number of functions to use - odd number recommended

    %determine period of radial function
    period = tVals(end) - tVals(1);
    
    %calculate offsets for functions
    offsets = linspace(-pi(), pi() - 2*pi()/functionCount, functionCount);
    
    %optomizationConstants
    residualError = .01;
    stepError = .0000001;
    
    %generate linearized effect matrix
    A = zeros(length(tVals), functionCount);
    for m = 1:length(tVals)
        for n = 1:functionCount
        
            A(m,n) = (sin(pi() / (2 * period) * tVals(m) + offsets(n)))^functionCount;
    
        end
    end
    
    residuals = transpose(yVals) + 1000;
    
    %calculate curve stats
    avgValue = sum(yVals) / length(yVals);
    MAD = sum(abs(avgValue - yVals)) / length(yVals);
    
    %iterate until under min error
    while(norm(residuals) / length(tVals) / MAD > residualError)
        %initial guess and error
        co = rand([functionCount, 1]);
        residuals = A * co - transpose(yVals);
    
        %calculate inital error
        error = norm(residuals) / length(tVals);
        previousError = norm(residuals + 10) / length(tVals);
    
        %iterate until minimization is no loner effective
        while(abs(error - previousError) > stepError)
    
            %minimize residuals
            ders = zeros(functionCount,1 );
    
            for i = 1:functionCount
                ders(i) = sum(A(:, i) .* residuals) / length(tVals);
            end
            
            co = co - ders(:) / functionCount;
            
            residuals = A * co - transpose(yVals);
    
            %calculate error
            previousError = error;
            error = norm(residuals) / length(tVals) / MAD;
    
        end
        %disp(error)
    
    end
end