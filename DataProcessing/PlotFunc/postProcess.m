function [kap1, error1, kap2, error2, kapBar11, error11, kapBar22, error22, ratio, kapB]= postProcess(simName, jointName, observeName, c1, c2)
[kap1, error1, kap2, error2, jointData, simData] = preProcess(simName, jointName, observeName); 

% fitting the error in the neighborhoold of buckling with same BC during loading and unloading
maxK = kap1(end);
minK = max(kap1(1), kap2(1));
minK = 0.3 * minK + 0.7 * maxK;

num = ceil(max(kap1)/0.001);
kapInt = linspace(minK, maxK, num);
[kap1, idx] = sort(kap1);
[kap1, ia, ~] = unique(kap1);
error1 = error1(idx);
error1 = error1(ia);
[kap2, idx] = sort(kap2);
[kap2, ia, ~] = unique(kap2);
error2 = error2(idx);
error2 = error2(ia);

error11 = interp1(kap1, error1, kapInt);
error22 = interp1(kap2, error2, kapInt);

% kap = [kap1; flip(kap2)];
% error = [error1; flip(error2)];
% [~, idx] = max(error);
% kap1 = kap(1:idx);
% kap2 = kap(idx:end);
% 
% error1 = error(1:idx);
% error2 = error(idx:end);

[error11, error22] = removePathError(jointData, simData, kapInt, error11, error22, 2, 10);


kap = [kapInt, flip(kapInt)];
error = [error11, flip(error22)];
[~, idx] = max(error);
kap11 = kap(1:idx);
kap22 = kap(idx:end);

error11 = error(1:idx);
error22 = error(idx:end);


    
idx = ~isnan(error11);
kap11 = kap11(idx);
error11 = error11(idx);

idx = ~isnan(error22);
kap22 = kap22(idx);
error22 = error22(idx);

plot(kap1, error1, 'b-');
hold on;
plot(kap2, error2, 'r-');  
plot(kap11, error11, 'go');
plot(kap22, error22, 'yo');
hold off;


errorMax = min([max(error11), max(error22)]);
errorAve = mean([error11, error22]);

alpha = 0.5;
gap = 10;
errorBar = alpha *errorMax + (1-alpha) * errorAve;

kapBar11 = kap11(abs(error11- errorBar) < (errorMax - errorBar)/gap);
kapBar22 = kap22(abs(error22- errorBar) < (errorMax - errorBar)/gap);

error11 = error11(abs(error11- errorBar) < (errorMax - errorBar)/gap);
error22 = error22(abs(error22- errorBar) < (errorMax - errorBar)/gap);


    
if isempty(kapBar11) || isempty(kapBar22)
    ratio = 1;
else
    idx1 = dbscan(kapBar11', 0.05 * max(kapInt), 1);
    if find(idx1 == 2)
        ratio =1 ;
    else
        idx2 = dbscan(kapBar22', 0.05 * max(kapInt), 1);
        if find(idx2 == 2)
            ratio = 1;
        else
            ratio = abs((mean(kapBar11) - mean(kapBar22))/max(kapInt));
        end
    end
end

% kapBar11 = mean(kapBar11);
% error11 = mean(error11);
% kapBar22 = mean(kapBar22);
% error22 = mean(error22);

alpha = 0.5;
errorBar = alpha *errorMax + (1-alpha) * errorAve;
kapfind = flip(kap1);
kapB = kapfind(find(flip(error1) < errorBar, 1));
    
% plot(kap1, error1);
% hold on;
% plot(kap2, error2);
% plot(kapBar11, error11, 'go');
% plot(kapBar22, error22, 'go');
% 
% drawnow();
end