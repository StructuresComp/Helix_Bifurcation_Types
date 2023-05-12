clc; clear all; close all;

%% define the exploring direction
S = [1, 1, 0];
S = S/norm(S);
Poisson = 0.5;
experiment = 1;

%% Compute the buckling point theoretically
dS = 0.01;
tol = 1e-5;
while true
    temp = dS * S;
    isstable = Helix_Configuration(temp(1),temp(2),temp(3),1.33);
    if ~isstable
        dSL = dS/2;
        dSR = dS;
        break;
    end
    dS = 2 * dS;
end

while abs(dSL - dSR) > tol
    dS = (dSL + dSR)/2;
    temp = dS * S;
    isstable = Helix_Configuration(temp(1),temp(2),temp(3),1.33);

    if isstable
        dSL = dS;
    else
        dSR = dS;
    end
end

kapB = dS * S(1);
tauB = dS * S(2);
omegaB = dS * S(3);

%% Running the simulation
c1 = S(2)/S(1);
c2 = S(3)/S(1);
command = sprintf(['./simDER option.txt -- c1 %f -- c2 %f -- Poisson %f',...
                  ' -- kapB %f -- experiment %d\n'], c1, c2, Poisson, kapB, experiment);
status = system(command);

%% if experiment, quit
if experiment ~= 0
    fprintf("Please do the motion planning with the output file\n");
    return;
end
%% Plot Simulation result
fileName = sprintf(['datafiles/simData_c1_%.5g_c2_%.5g_Poisson_%.5g_kapB',...
                   '_%.5g_exp_%d.txt'], c1, c2, Poisson, kapB, experiment);
data = importdata(fileName);

% get all data
kap = data(:,1);
error = data(:, 10);
type = data(:, 12);
% get loading data
kap1 = kap(type == 0);
error1 = error(type == 0);
% get unloading data
kap2 = kap(type == 2);
error2 = error(type == 2);

plot(kap * sqrt(1+c1^2+c2^2), error, 'k--');
hold on;
plot(kap1 * sqrt(1+c1^2+c2^2), error1, 'bo');
plot(kap2 * sqrt(1+c1^2+c2^2), error2, 'r^');
xlabel("Exploring distance along $\mathbf S$, $\lVert \mathbf S \rVert$", 'intepreter', 'latex');
ylabel("Difference between the rod configuration and the prescribed helix, $e$", intepreter, 'latex');

