function pointsM = fittedPoints(kap0, pointsM0, kap)
diffKap = diff(kap0);
diffKap = [0; diffKap];

idx = find(diffKap > 0);

kap0 = kap0(idx);
pointsM0 = pointsM0(idx, :);

pointsM = interp1(kap0, pointsM0, kap, 'linear', 'extrap');



end