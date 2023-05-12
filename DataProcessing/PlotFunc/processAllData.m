% clc; clear all; clsose all;

%% Process parameters
% withTwist means processing the exp data with external twisting moment
withTwist = false;


%% 
dinfo = dir("../Observations/*.txt");


preFix = "..";

Kappa = [];
Type = [];
C11 = []; C22 = [];
for i = 1:length(dinfo)
    name = dinfo(i).name;
    C = regexp(name,'[+-]?\d+\.?\d*','match');
    c1 = str2double(C{2}); c2 = str2double(C{4});
    if withTwist
        if c2 == 0
            continue;
        end
    else
        if c2~=0
            continue;
        end
    end
    
    
    simName = sprintf("%s/Simulations/simData_c1_%.6g_c2_%.6g.txt", preFix, c1, c2);
    jointName = sprintf("%s/Joints/c1_%.6g_c2_%.6g.txt", preFix, c1, c2);
    observeName = sprintf("%s/Observations/test_c1_%g_c2_%g.txt", preFix, c1, c2);

    S = [1, c1, c2];
    S = S/norm(S);
    
    
    
    %% preProcess and visualize data
    [kap1, error1, kap2, error2] = preProcess(simName, jointName, observeName); 
    factor = sqrt(1+c1^2+c2^2);
    
    %% post process the data to get the classification point
    [kap1, error1, kap2, error2, kapBar1, errorBar1, kapBar2, errorBar2, ratio, kapB]= postProcess(simName, jointName, observeName, c1, c2);
    
    %% Decide if the buckling is the supercritical or subcritical
    if ratio > 0.05
        type =1;
    else
        type = 0;
    end
    
    errorT = max([errorBar1, errorBar2]);
    errorB = min([errorBar1, errorBar2]);
%     errorB = (errorBar1 + errorBar2)/2 - 2*abs(errorBar1 -errorBar2);

    h(1) = plot(kap1 * factor, error1, 'b-');
    hold on;
    h(2) = plot(kap2 * factor, error2, 'r-');  
    
    legendC = {'Loading', 'Unloading'};
    
    gap = 1;
    
    if ~isempty(kapBar1) || ~isempty(kapBar2)
         h(3) = plot(1.05 * kap1 * factor, errorT .* ones(length(kap1), 1), 'k--');
         h(3) = plot(1.05 * kap1 * factor, errorB .* ones(length(kap1), 1), 'k--');
         if ~isempty(kapBar1)
             h(3) = plot(1.05 * kap1 * factor, errorT .* ones(length(kap1), 1), 'k--');
             h(4) = plot(kapBar1 * factor, errorBar1, 'o','markerfacecolor','g',...
                 'markeredgecolor','none','markersize',4);
             h(5) = plot(mean(kapBar1) * factor, mean(errorBar1), 'kx');
             legendC = {'Loading','Unloading', 'Test Region', 'Detected points', 'Classification nodes'};
         end
         if ~isempty(kapBar2)
              h(4) = plot(kapBar2 * factor, errorBar2, 'o','markerfacecolor','g',...
                  'markeredgecolor','none','markersize',4);
              h(5) = plot(mean(kapBar2) * factor, mean(errorBar2), 'kx');
              legendC = {'Loading','Unloading', 'Test Region', 'Detected points', 'Classification nodes'};
         end
    end    
        
    hold off;
    
    xlabel('Dis along searching dir, $||\mathbf{S}||$','interpreter','latex');
    ylabel('Error w.r.t. helix, $e$','interpreter','latex');
    title(sprintf('Exploring Direction: S = [%f, %f, %f]', S(1), S(2), S(3)), ...
        'interpreter', 'latex');
        
    legend(h, legendC,'location','northwest')

    
    fprintf("Type is %d, with diffrence between loading and unloading is %f. (1 is subcritical, 0 is supercritical)\n", type, ratio);
    Type = [Type; type];
    Kappa = [Kappa; kapB]; 
    C11 = [C11; c1];
    C22 = [C22; c2];
    drawnow();
    clear h;

end

%% Visualize Experimental Results
Omega = Kappa .* C22;
Tau = Kappa .* C11;
idx = find(Type == 0);

if withTwist
    h(1) = plot3(Kappa(idx), Tau(idx), Omega(idx), 'bsquare');
    hold on;
    idx1 = find(Type == 1);
    h(2) = plot3(Kappa(idx1), Tau(idx1), Omega(idx1), 'rsquare');

    xlabel('\kappa')
    ylabel('\tau')
    zlabel('\omega')

    axis([0.8*pi 3*pi 0.5*pi 2.2*pi -1.8*pi 1.8*pi])
    daspect([1 1 1])
    light('Position',[20 -20 10],'Style','infinite')
    set(gcf,'color','w')
    view(120,5)

    xticks([0*pi 1*pi 2*pi 3*pi])
    xticklabels({'$0\pi$','$\pi$','$2\pi$','$3\pi$'})
    yticks([0 pi 2*pi 3*pi])
    yticklabels({'$0$','$\pi$','$2\pi$','$3\pi$'})
    zticks([-2*pi -pi 0  pi 2*pi])
    zticklabels({'$-2\pi$','$-\pi$', '$0$', '$\pi$','$2\pi$'})
    daspect([1 1 1])
else
    h(1) = plot(Tau(idx), Kappa(idx), 'bsquare');
    hold on;
    plot(-Tau(idx), Kappa(idx), 'bsquare');
    idx = find(Type == 1);
    h(2) = plot(Tau(idx), Kappa(idx), 'rsquare');
    plot(-Tau(idx), Kappa(idx), 'rsquare');

    axis equal;


    set(gcf,'color','w')
    axis([-3*pi 3*pi 0 4*pi])

    xticks([-4*pi -2*pi 0 2*pi 4*pi])
    xticklabels({'$-3\pi$','$-2\pi$','$0$','$2\pi$','$3\pi$'})
    yticks([0 pi 2*pi 3*pi])
    yticklabels({'$0$','$\pi$','$2\pi$','$3\pi$'})
    ylabel('\kappa')
    xlabel('\tau') 
end

legend(h, {'Supercritical','Subcritical'},'location','northwest')
set(gca,'fontsize',30,'TickLabelInterpreter','latex');
set(gcf,'color','w')

