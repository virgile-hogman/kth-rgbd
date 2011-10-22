clear
clf

M = dlmread('~/data_out/poses_initial.dat','\t');
figure
hold on
plot_matrix(M);
title('Poses - initial graph');

M = dlmread('~/data_out/poses_optimized.dat','\t');
figure
hold on
plot_matrix(M);
title('Poses - optimized graph');
