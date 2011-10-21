clc
clear all
d=load('cvap_tr14_floor7-rlm.txt');
a=load('~/data_out/poses.dat');
%a=[0 0];

clf, hold on
title('CVAP 7th floor');
xlabel('x (meters)');
ylabel('y (meters)');

text(2,0.5,'R1', 'FontSize',10)
text(8,0.5,'R2', 'FontSize',10)
text(13.5,11.8,'R3', 'FontSize',10)
text(16.5,0.5,'R4', 'FontSize',10)

%figure
% plot( -a(:,9),'--r');
% title('Vertical drift');

% read x,y coord as z,x
a = [a(:,13) a(:,5)];

% correct initial pose and orientation
angle = -6;
offx = -3.9;
offy = 3;

%rotation
R = [cosd(angle) -sind(angle); sind(angle) cosd(angle)];
a = (R * a')';

%translation
T = [offx*ones(size(a,1),1) offy*ones(size(a,1),1)] ;
a = a + T;

 for k=1:size(d,1)
     plot(d(k,[2 4]), d(k,[1 3]))
 end
 %for k=1:size(a,1)-1
 %     p = scatter(-a(k,1), a(k,2), 'filled');
 %end
 for k=1:size(a,1)
     plot( -a(k,1), a(k,2),':r', 'LineWidth', 25);
 end
 %plot( -a(:,1), a(:,2),'.r', 'MarkerSize',5);
      
 set(gca,'xdir','reverse');
 xlim([-5 45]);
 ylim([-5 14]);
 
 total = 0;
  for k=1:size(a,1)-1
      total = total +  sqrt((a(k,1) - a(k+1,1))^2 + (a(k,2) -a(k+1,2))^2);
  end
  
  if (total > 0)
    dist = sprintf('Total Distance = %.2f m', total);
    text(30,-3, dist, 'FontSize',10)
    %text('\leftarrowend', 'FontSize',10)
  end
  
  