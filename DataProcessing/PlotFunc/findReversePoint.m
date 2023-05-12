function [idx1, idx] = findReversePoint(kap0, joints0, expjoints)

[val, idx] = max(kap0);

joints = joints0(idx, :);

je = sum((expjoints - joints).^2, 2);
[val, idx1] = min(je);


end