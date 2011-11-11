close all
clf
clc
clear all

sPathData = '~/Documents/stats/';

% Initial poses
S1=load(strcat(sPathData,'stats_sift_default.log'));
S2=load(strcat(sPathData,'stats_surf_big.log'));
S3=load(strcat(sPathData,'stats_surf_default.log'));

LABEL1 = 'SIFT 6 - ';
LABEL2 = 'SURF 6/300 - ';    % 6 octaves, Hessian Threshold 300
LABEL3 = 'SURF 3/500 - ';    % 3 octaves, Hessian Threshold 500

hold on
plot(S1(:,2),'b')
plot(S2(:,2),'r')
plot(S3(:,2),'k')
xlabel('Frames')
ylabel('Number of features')
s1 = sprintf('%s\\mu = %.f', LABEL1, mean(S1(:,2)));
s2 = sprintf('%s\\mu = %.f', LABEL2, mean(S2(:,2)));
s3 = sprintf('%s\\mu = %.f', LABEL3, mean(S3(:,2)));
legend(s1,s2,s3);
xlim([0 size(S1,1)]);

figure
hold on
plot(S1(:,3),'b')
plot(S2(:,3),'r')
plot(S3(:,3),'k')
xlabel('Frames')
ylabel('time (ms)')
s1 = sprintf('%s\\mu = %.f ms', LABEL1, mean(S1(:,3)));
s2 = sprintf('%s\\mu = %.f ms', LABEL2, mean(S2(:,3)));
s3 = sprintf('%s\\mu = %.f ms', LABEL3, mean(S3(:,3)));
legend(s1,s2,s3);
xlim([0 size(S1,1)]);

figure
hold on
M1=S1(:,3)./S1(:,2);
M2=S2(:,3)./S2(:,2);
M3=S3(:,3)./S3(:,2);
plot(M1,'b')
plot(M2,'r')
plot(M3,'k')
xlabel('Frames')
ylabel('time by feature (ms)')
s1 = sprintf('%s\\mu = %.3f ms', LABEL1, mean(M1))
s2 = sprintf('%s\\mu = %.3f ms', LABEL2, mean(M2))
s3 = sprintf('%s\\mu = %.3f ms', LABEL3, mean(M3))
legend(s1,s2,s3);
xlim([0 size(S1,1)]);
