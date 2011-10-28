close all

%sPathData = '~/results/set4_sift/';
sPathData = '~/data_out/';

display_laser=0;

figure

% Initial poses
P=load(strcat(sPathData,'poses_initial.dat'));

subplot(3,1,1)
plot(P(:,5),'--r')
xlabel('X')
ylabel('meters')
subplot(3,1,2)
plot(P(:,9),'--r')
xlabel('Y')
ylabel('meters')

Y=P(:,9);

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
plot(P(:,13),'--r')
xlabel('Z')
ylabel('meters')

% Optimized poses
% P2=load(strcat(sPathData,'poses_optimized.dat'));
% 
% subplot(3,1,1)
% hold on
% plot(P2(:,5),'b')
% legend('rgbd initial','rgbd optimized','Location','BestOutside');
% subplot(3,1,2)
% hold on
% plot(P2(:,9),'b')
% legend('rgbd initial','rgbd optimized','Location','BestOutside');
% subplot(3,1,3)
% hold on
% plot(P2(:,13),'b')
% legend('rgbd initial','rgbd optimized','Location','BestOutside');

if (display_laser ~= 0)

    L=load('cureslampose-set4.tdf');

    subrange=round(linspace(1,size(L,1),size(T2,1)));

    subplot(3,1,1)
    hold on
    plot(L(subrange,9),'k-.');
    legend('rgbd initial','rgbd optimized','laser','Location','BestOutside');
    subplot(3,1,3)
    hold on
    plot(-L(subrange,10),'k--')
    legend('rgbd initial','rgbd optimized','laser','Location','BestOutside');

end

% transfo
% T=load(strcat(sPathData,'transfo.dat'));
% 
% figure
% title('Transfo')
% subplot(3,1,1)
% plot(T(:,6),'r')
% xlabel('X')
% ylabel('meters')
% subplot(3,1,2)
% plot(T(:,10),'r')
% xlabel('Y')
% subplot(3,1,3)
% ylabel('meters')
% plot(T(:,14),'r')
% xlabel('Z')
% ylabel('meters')
% 
% figure
% hold on
% pos = eye(4);
% M = [];
% for i = (1:size(T,1))
%     t = [T(i, 3:18)];
%     t = reshape(t,4,4)
%     pos = inv(t) * pos;
%     M = [M; pos];
% end
% plot(M(:,4));
