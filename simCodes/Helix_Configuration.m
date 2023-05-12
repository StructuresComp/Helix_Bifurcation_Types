% This function computes configurations of a Kirchhoff elastic rod with
% helical centerlines and determines their stability.  
% The rod is assumed to be inextensible, unshearable, isotropic, and
% uniform, and there are no body forces acting on the rod.
% It is assumed that the length of the rod has been nondimensionalized to
% be 1.  
% ~Andy Borum, 1/14/2020

function [isstable,s,r,R] = Helix_Configuration(k,t,w,c)
% This function computes helical configurations of a Kirchhoff elastic rod
% and determines if they are stable.
% INPUTS: k - curvature of helical rod's centerline
%         t - torsion of helical rod's centerline
%         w - twist of rod about the helical centerline
%         c - ratio of rod's twisting to bending stiffness
%
% OUTPUTS: isstable - variable determining stability of rod, the rod is
%                     stable if isstable=1 and is unstable if
%                     isstable=0
%          s - 1xn vector of arc lengths along rod, where s(1)=0 and s(n)=1
%          r - 3xn vector containing points along rod's centerline, where
%              r(:,0)=0 and r(:,i) corresponds to the centerline position 
%              at arc length s(i)
%          R - 3x3xn structure describing the orientation of rod's
%              cross-sections, where R(:,:,1)=eye(3) and R(:,:,i) is a 3x3
%              rotation matrix describing the orientation of the
%              cross-section at arc length s(i)
%              Note: R(:,:,1) can be chosen so that the helix is oriented
%              along a desired axis, see below

% Arc length interval
L = [0 1];

% Initial conditions for torques
m0 = [w k 0]; 

% Initial conditions for forces
n0 = (w-t)*[t k 0]; 

% Position of centerline at arclength s=0
r0 = [0 0 0];

% Orientation of cross-section at arclength s=0
R0 = eye(3);
% ~or~
% Uncomment the code below if you want the helix to be oriented along a
% desired axis
% helix_ax = [0 0 1]; % Desired axis of helix
% rot_vec = cross(n0,helix_ax); % Rotation vector
% rot_ang = atan2(norm(cross(n0,helix_ax)),dot(n0,helix_ax)); % Rotation angle
% R0 = expm(rot_ang*Hat(rot_vec)/norm(rot_vec)); % Orientation of cross-section at arclength s=0


% Combine r0 & R0 into a single centerline/cross-section orientation matrix
% (elemnet of the special Euclidean group SE(3))
q0 = [R0 r0'; 0 0 0 1];

% Initial conditions for stability equations
M0 = eye(6); J0 = zeros(6,6); 

% Initial condition vector
X0 = [reshape(q0(1:3,:)',1,12) m0 n0 reshape(M0',1,36) reshape(J0',1,36)]; 

% Vector of twisting and bending stiffnesses
C = [c 1 1];

% Solve system of ODEs
options = odeset('RelTol',10^-10,'AbsTol',10^-10,'MaxStep',.01,'Events',@CP_Func);
[s,sol,Tconj] = ode45(@(s,X) Eqs(s,X,C),L,X0,options);

% Construct structure containing q
q=[sol(:,1:12) zeros(length(s),3) ones(length(s),1)];
q=permute(reshape(q',4,4,length(s)),[2 1 3]);

% Get r and R
r = reshape(q(1:3,4,:),3,length(s),1);
R = q(1:3,1:3,:);

% Determine if the rod is stable 
if isempty(Tconj) || Tconj(1)>=1-1e-6
    isstable=true;
else
    isstable=false;
end
    
% Plot helix configuration
% plot3(r(1,:),r(2,:),r(3,:),'b-')
% daspect([1 1 1])
% xlabel('x')
% ylabel('y')
% zlabel('z')
% title('Helix Configuration')
% set(gca,'FontSize',20)
% set(gcf,'color','w')

end





function [detJ,isterminal,direction] = CP_Func(s,X)
% This function computes the determinant of the matrix J(t).  The ODE
% solver within the function Helix_Configuration tracks when det(J(t))=0.
% INPUTS: s - current arc length.
%         X - solution of equilibrium and stability equations at 
%             arc length t.
%
% OUTPUTS: detJ - value of det(J(t)).
%          isterminal - equal to 0, indicating that the ODE solver should 
%                       not terminate when detJ=0.
%          direction - equal to [], indicating that we don't care if
%                      det(J(t)) is increasing or decreasing when it 
%                      crosses 0.

isterminal = 0;
direction = [];

% If t=0, then det(J(0))=0, but this isn't a conjugate point, so we only
% check det(J(0)) when t>0
if s == 0
    detJ = 1;
else
    % Get the matrix J
    J = reshape(X(55:90),6,6)';
    % Compute det(J(t))
    detJ = det(J);
end

end





function dX = Eqs(~,X,C)
% This function contains the equilibrium and stability ODEs.
% INPUTS: X - solution of equilibrium and stability equations at 
%             arc length s.
%         C - 1x3 vector of stiffnesses.
%
% OUTPUTS: dX - derivative (with respect to arc length) of X

% Get centerline/cross-section orientation matrix
q=[reshape(X(1:12),4,3)'; 0 0 0 1];

% Get moments and forces
m = X(13:15)';
n = X(16:18)';

% Get stability equation solution
M=reshape(X(19:54),6,6)';
J=reshape(X(55:90),6,6)';

% Compute twisting/bending strains u and axial/shear strains v
k = 1./C;
u = m.*k;
v = [1 0 0];

% Differential equations for centerline/cross-section orientation matrix
dq=q*[Hat(u) v'; 0 0 0 0];

%  Differential equations for moments and forces
dm = cross(m,u)+cross(n,v);
dn = cross(n,u);

% Construct the coefficient matrices in the stability equations
F11 = [0                m(3)*(k(3)-k(2))  m(2)*(k(3)-k(2));...
       m(3)*(k(1)-k(3)) 0                 m(1)*(k(1)-k(3));...
       m(2)*(k(2)-k(1)) m(1)*(k(2)-k(1))  0];
F21 = Hat(n)*diag(k); 
F = [F11 -Hat(v); F21 -Hat(u)];
G = diag([k 0 0 0]);
H = [-Hat(u) zeros(3,3); -Hat(v) -Hat(u)];

%  Differential equations for stability
dM = F*M;
dJ = G*M+H*J;

% Collect all derivatives into one vector dX
dX = [reshape(dq(1:3,1:4)',1,12) dm dn reshape(dM',1,36) reshape(dJ',1,36)]';

end





function uhat = Hat(u)
% This function computes the angular rotation matrix uhat corresponding to
% a 3-dimensional vector u.
% INPUTS: u - 1x3 or 3x1 vector
%
% OUTPUTS: uhat- 3x3 angular rotation matrix corresponding to u

uhat = [0     -u(3) u(2);...
     u(3)  0     -u(1);...
     -u(2) u(1)  0];

end