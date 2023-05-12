function kap = fittedKap(kap0, joints0, expjoints)

kap = zeros(size(expjoints, 1), 1);

for i = 1:size(expjoints, 1)
    tmpJoints = expjoints(i, :);
    diffJoints = joints0 - tmpJoints;
    je = sqrt(sum(diffJoints.^2, 2));
    [~, idx] = min(je);

    if idx == 1
        idx = idx + 1;
    end
    if idx == size(diffJoints, 1)
        idx = idx -1;
    end
    d1 = je(idx - 1);
    d2 = je(idx);
    d3 = je(idx + 1);

    S = (d1 + d2 + d3) * 2;
    kap1 = kap0(idx - 1);
    kap2 = kap0(idx);
    kap3 = kap0(idx + 1);

    kap(i) = kap1*(d2 + d3)/S + kap2*(d1+d3)/S + kap3*(d1+d2)/S;

end

end