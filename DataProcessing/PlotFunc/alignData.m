function [detectM, pointsM] =alignData(pointsM, detectM, idx)

coord3D = detectM(1:idx,:)';
coord3D_ref = pointsM(1:idx, :)';

detectM = detectM';
pointsM = pointsM';

aveCoord = mean(coord3D, 2);

coord3D = coord3D - aveCoord;
detectM = detectM - aveCoord;

aveCoord = mean(coord3D_ref, 2);
coord3D_ref = coord3D_ref - aveCoord;
pointsM = pointsM - aveCoord;


H = coord3D * (coord3D_ref');

[u, s, vh] = svd(H);

% S = [s(1), 0, 0;
%      0, s(2), 0;
%      0, 0, s(3)];


signM = (u * vh')';

d = sign(det(signM));

S = [1, 0, 0;
     0, 1, 0;
     0, 0, d];
R = (u * S * vh')';

detectM = R * detectM;

% plot3(detectM(1,1:idx), detectM(2,1:idx), detectM(3,1:idx),'o');
% hold on;
% plot3(pointsM(1,1:idx), pointsM(2,1:idx), pointsM(3,1:idx), 'o');
% axis equal;

detectM = detectM';
pointsM = pointsM';

end