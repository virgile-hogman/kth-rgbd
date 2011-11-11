clc
clear all
close all
d=load('cvap_tr14_floor7-rlm.txt');
L=load('cureslampose-set4.tdf');

sPathData = '~/results/set4_surf/';
sLegend1 = 'rgbd (SURF)';
sLegend2 = 'laser';

% Initial poses
P_init=load(strcat(sPathData,'poses_initial.dat'));
P_optim=load(strcat(sPathData,'poses_optimized.dat'));

%a=[0 0];
% read x,y coord as z,x
X_opt = P_optim(:,5);
Y_opt = P_optim(:,9);
Z_opt = P_optim(:,13);
Poses_2d_optim = [X_opt -Z_opt];
X_init = P_init(:,5);
Y_init = P_init(:,9);
Z_init = P_init(:,13);
Poses_2d_init = [X_init -Z_init];
% correct initial pose and orientation
angle = -11;
offx = 2.5;
offy = 4.0;

%start at same origin as laser
offx = L(1,9);
offy = L(1,10);

figure
hold on
plot( Y_opt-Y_opt(1),'r');
%line( [0 size(Y,1)], [Y(1) Y(1)], 'Color', 'k', 'LineStyle','--');
line( [0 size(Y_opt,1)], [0 0], 'Color', 'k', 'LineStyle','--');
title(strcat('Vertical drift - ',sLegend1));
xlim([0 size(Y_opt,1)]);
ylabel('meters');
xlabel('position');


figure
clf, hold on
title('CVAP 7th floor');
xlabel('x (meters)');
ylabel('y (meters)');

%label rooms
%text(2,0.5,'R1', 'FontSize',10)
%text(8,0.5,'R2', 'FontSize',10)
%text(13.5,11.8,'R3', 'FontSize',10)
%text(16.5,0.5,'R4', 'FontSize',10)
text(0.2,1.5,'R1', 'FontSize',10)
text(0.2,7.5,'R2', 'FontSize',10)
text(11.8,13,'R3', 'FontSize',10)
text(0.2,16,'R4', 'FontSize',10)

%set(gca,'xdir','reverse');
%xlim([-5 45]);
ylim([-5 45]);

%rotation
Rot = [cosd(angle) -sind(angle); sind(angle) cosd(angle)];
Poses_2d_optim = (Rot * Poses_2d_optim')';
%translation
Tra = [offx*ones(size(Poses_2d_optim,1),1) offy*ones(size(Poses_2d_optim,1),1)] ;
Poses_2d_optim = Poses_2d_optim + Tra;
 
%rotation
Rot = [cosd(angle) -sind(angle); sind(angle) cosd(angle)];
Poses_2d_init = (Rot * Poses_2d_init')';
%translation
Tra = [offx*ones(size(Poses_2d_init,1),1) offy*ones(size(Poses_2d_init,1),1)] ;
Poses_2d_init = Poses_2d_init + Tra;

plot( Poses_2d_optim(:,1), Poses_2d_optim(:,2),'r', 'LineWidth',1);

total = compute_distance(Poses_2d_optim(:,1:2));
if (total > 0)
    sDist = sprintf('Total Distance = %.2f m', total);
    text(8,0, sDist, 'FontSize',10,'Color',[1 0 0])
end

plot(L(:,9),L(:,10),'k--')
legend(sLegend1, sLegend2);

total = compute_distance(L(:,9:10));
if (total > 0)
    sDist = sprintf('Total Distance = %.2f m', total);
    text(8,-2, sDist, 'FontSize',10)
end

% map
for k=1:size(d,1)
     plot(d(k,[1 3]), d(k,[2 4]),'b')
end

[xd,zd] = plot_poses_xyz([Poses_2d_init(:,1) Y_init Poses_2d_init(:,2)], [Poses_2d_optim(:,1) Y_opt Poses_2d_optim(:,2)]);
diff = sqrt(xd.^2 + zd.^2);