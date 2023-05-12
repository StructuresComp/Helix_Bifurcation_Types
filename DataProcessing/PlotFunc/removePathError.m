function [error1, error2] = removePathError(jointData, simData, kapInt, error1, error2, dense, gap)

sawyer = importrobot('sawyer1.urdf');
sawyer.DataFormat = 'column';
numJoints = 8; % Number of joints in robot

computePath = [];
idx = linspace(1, length(jointData), dense*length(jointData));
jointData1 = interp1((1:length(jointData))', jointData, idx, 'cubic');

for i = 1:length(jointData1)
    joint0 = jointData1(i, :);
    joint = [joint0(1); 0; joint0(2:end)'; 0; 0];
    transform = getTransform(sawyer,joint,'right_motor_tip', 'base');
    points = transform(1:3, 4)';
    computePath = [computePath; transform(1:3, 4)'];
end

computePath = computePath - computePath(1,:);
simRef = simData(:, 4:6);
simRef = interp1((1:length(jointData))', simRef, idx);
simRef = simRef - simRef(1, :);


diffPath = simRef - computePath;
diffPath = sqrt(sum(diffPath.^2, 2));

idx = find(diffPath > 0.01);
idx = ceil(idx/dense);

% span idx    
idx = unique(idx);
if ~isempty(idx)
    %% blur those data 
    kapBlur = simData(idx, 1);
    Idx = dbscan(kapBlur, max(kapInt)/gap, 1);
    i = 1;
    while ~isempty(find(Idx==i, 1))
        kapTmp = kapBlur(Idx == i);
        
        if abs(kapTmp - max(kapInt))/max(kapInt) < 0.1
            i = i + 1;
            continue;
        end
        % blur the data 
        idx = find(and(kapInt > min(kapTmp)-max(kapInt)/gap, kapInt < max(kapTmp) + max(kapInt)/gap));
        tempidx2 = find(and(kapInt > max(kapTmp) + max(kapInt/gap), kapInt < max(kapTmp) + 2*max(kapInt/gap)));
        tempidx1 = find(and(kapInt < min(kapTmp) - max(kapInt/gap), kapInt > min(kapTmp) - 2*max(kapInt/gap)));
        if ~isempty(tempidx1)
%             error1(idx) = mean(error1(tempidx1));
            error1(idx) = nan;
        end
        if ~isempty(tempidx2)
%             error2(idx) = mean(error2(tempidx2));
            error2(idx) = nan;
        end
        i = i +1;
    end
end


end