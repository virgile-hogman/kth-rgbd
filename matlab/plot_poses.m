clear
clf

sPathData = '~/results/set4_surf/';

M = dlmread(strcat(sPathData,'poses_initial.dat'),'\t');
figure
hold on
plot_matrix(M);
title('Poses - initial graph');

M = dlmread(strcat(sPathData,'poses_optimized.dat'),'\t');
figure
hold on
plot_matrix(M);
title('Poses - optimized graph');
