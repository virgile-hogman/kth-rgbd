clc
clear all
close all
d=load('cvap_tr14_floor7-rlm.txt');
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


% map
for k=1:size(d,1)
     plot(d(k,[1 3]), d(k,[2 4]),'b')
 end
 