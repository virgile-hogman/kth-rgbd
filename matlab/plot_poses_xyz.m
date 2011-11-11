function [xdiff, zdiff] = plot_poses_xyz(P_init,P_optim,L)

    if (nargin>2)
        display_laser=1;
    else
        display_laser=0;
    end
    xdiff=0; zdiff=0;
    %close all
    %
    %sPathData = '~/results/set4_sift/';
    %sPathData = '~/rgbd_prod/';
    % Initial poses
    %P=load(strcat(sPathData,'poses_initial.dat'));
    % Optimized poses
    %P2=load(strcat(sPathData,'poses_optimized.dat'));
    %if (display_laser ~= 0)
    %    L=load('cureslampose-set4.tdf');
    %end

    figure
    subplot(3,1,1)
    plot(P_init(:,1),'--r')
    xlabel('X')
    ylabel('meters')
    subplot(3,1,2)
    plot(P_init(:,2),'--r')
    xlabel('Y')
    ylabel('meters')

    Y=P_init(:,2);

    imin = find(min(Y) == Y);		% Find the index of the min and max
    imax = find(max(Y) == Y);
    text(0,Y(imin),[' min = ',num2str(Y(imin))],...
        'VerticalAlignment','top',...
        'HorizontalAlignment','left',...
        'FontSize',10,'Color',[0 0 1])
    text(0,Y(imax),[' max =  ',num2str(Y(imax))],...
        'VerticalAlignment','bottom',...
        'HorizontalAlignment','left',...
        'FontSize',10,'Color',[0 0 1])

    subplot(3,1,3)
    plot(P_init(:,3),'--r')
    xlabel('Z')
    ylabel('meters')

    subplot(3,1,1)
    hold on
    plot(P_optim(:,1),'b')
    legend('rgbd initial','rgbd optimized','Location','BestOutside');
    subplot(3,1,2)
    hold on
    plot(P_optim(:,2),'b')
    legend('rgbd initial','rgbd optimized','Location','BestOutside');
    subplot(3,1,3)
    hold on
    plot(P_optim(:,3),'b')
    legend('rgbd initial','rgbd optimized','Location','BestOutside');

    if (display_laser ~= 0)
        subrange=round(linspace(1,size(L,1),size(P_optim,1)));

        xdiff = P_optim(:,1) - L(subrange,9);
        zdiff = P_optim(:,3) - L(subrange,10);
        
        subplot(3,1,1)
        hold on
        plot(L(subrange,9),'k-.');
        plot(abs(xdiff),'k')
        legend('rgbd initial','rgbd optimized','laser','Location','BestOutside');
        subplot(3,1,3)
        hold on
        plot(L(subrange,10),'k--')
        plot(abs(zdiff),'k')
        legend('rgbd initial','rgbd optimized','laser','Location','BestOutside');

    end
    
    
end