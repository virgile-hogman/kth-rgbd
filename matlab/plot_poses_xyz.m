close all

%sPathData = '~/data_out/set4_sift/';
sPathData = '~/data_out/set4_surf/';

figure

% Initial poses
T=load(strcat(sPathData,'poses_initial.dat'));

subplot(3,1,1)
plot(T(:,5),'--r')
xlabel('X')
subplot(3,1,2)
plot(T(:,9),'--r')
xlabel('Y')

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
plot(T(:,13),'--r')
xlabel('Z')

% Optimized poses
T2=load(strcat(sPathData,'poses_optimized.dat'));

subplot(3,1,1)
hold on
plot(T2(:,5),'b')
legend('Initial','Optimized','Location','BestOutside');
subplot(3,1,2)
hold on
plot(T2(:,9),'b')
legend('Initial','Optimized','Location','BestOutside');
subplot(3,1,3)
hold on
plot(T2(:,13),'b')
legend('Initial','Optimized','Location','BestOutside');

C=load('cureslampose-set4.tdf');

subrange=round(linspace(1,size(C,1),size(T2,1)));
 
subplot(3,1,1)
hold on
plot(C(subrange,9),'k-.');
legend('Initial','Optimized','Cure','Location','BestOutside');
subplot(3,1,3)
hold on
plot(-C(subrange,10),'k--')
legend('Initial','Optimized','Cure','Location','BestOutside');

