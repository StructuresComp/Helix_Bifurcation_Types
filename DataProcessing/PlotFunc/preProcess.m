function [kap1, error1, kap2, error2, jointData, simData]= preProcess(simName, jointName, observeName)
simData = importdata(simName);
jointData = importdata(jointName);
expData = importdata(observeName);

kap0 = simData(:, 1);
tau0 = simData(:, 2);
error0 = simData(:, end-4);
pointsM0 = simData(:, 7:9);

expjoints = expData(:, 1:7);
coord3D = expData(:, 8:10);

[idxRev, idxref] = findReversePoint(kap0, jointData, expjoints);
kap = fittedKap(kap0, jointData, expjoints);
tau = fittedKap(tau0, jointData, expjoints);
pointsM = fittedPoints(kap0, pointsM0, kap);


gap = 2;
idxRev = ceil(idxRev/gap);
coord3D = coord3D(1:gap:end, :);
pointsM = pointsM(1:gap:end, :);
kap = kap(1:gap:end);
tau = tau(1:gap:end);

idx = ceil(idxRev/1.1);
[coord3D1, pointsM1] = alignData(pointsM, coord3D, idx);


diffM = pointsM1 - coord3D1;
c = 1./(kap.^2 + tau.^2);
r = 0.51 * c .* kap;

error = sqrt(sum(diffM.^2, 2));
error = error./r;


[val, idx] = max(kap);
kap1 = kap(1:idx);
error1 = error(1:idx);

kap2 = kap(idx:end);
error2 = error(idx:end);

[~, idx] = max(error1);
kap1 = kap1(1:idx);
error1 = error1(1:idx);

% prune kap2
idx = find(kap2 < kap1(end));
kap2 = kap2(idx);
error2 = error2(idx);
idx = find(error2 > 0.5 * error1(end), 1, 'first');
kap2 = kap2(idx:end);
error2  = error2(idx:end);
kap2 = flip([kap1(end); kap2]);
error2 = flip([error1(end); error2]);

end