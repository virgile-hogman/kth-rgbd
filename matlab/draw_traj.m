clc
clear all
close all
d=load('cvap_tr14_floor7-rlm.txt');
L=load('cureslampose-set4.tdf');

sPathData = '~/results/set4_surf/';
sLegend1 = 'rgbd (SURF)';
sLegend2 = 'laser';

% Initial poses
Poses=load(strcat(sPathData,'poses_optimized.dat'));

%a=[0 0];
% read x,y coord as z,x
X = Poses(:,5);
Y = Poses(:,9);
Z = Poses(:,13);
Poses = [X -Z];
% correct initial pose and orientation
angle = -11;
offx = 2.5;
offy = 4.0;

%start at same origin as laser
offx = L(1,9);
offy = L(1,10);

figure
hold on
plot( Y-Y(1),'r');
%line( [0 size(Y,1)], [Y(1) Y(1)], 'Color', 'k', 'LineStyle','--');
line( [0 size(Y,1)], [0 0], 'Color', 'k', 'LineStyle','--');
title(strcat('Vertical drift - ',sLegend1));
xlim([0 size(Y,1)]);
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
Poses = (Rot * Poses')';
%translation
Tra = [offx*ones(size(Poses,1),1) offy*ones(size(Poses,1),1)] ;
Poses = Poses + Tra;
 
plot( Poses(:,1), Poses(:,2),'r', 'LineWidth',1);

total = compute_distance(Poses(:,1:2));
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


 