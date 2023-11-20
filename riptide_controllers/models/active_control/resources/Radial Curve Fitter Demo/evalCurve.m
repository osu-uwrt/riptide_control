function val = evalCurve(co,t)
    %co - radial coefficents
    %t - the position to evaluate on a scale of zero to one

    val = 0;

    functionCount = length(co);
    offsets = linspace(-pi(), pi() * (1 - 2 / functionCount), functionCount);

    for i = 1:length(co)
        val = val + co(i) * sin(pi() / 2 * t + offsets(i)) ^ functionCount;
    end

    disp(offsets)
end