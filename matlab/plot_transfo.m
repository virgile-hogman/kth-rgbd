T=load('~/results/transfo.dat');

figure
title('Transformations');
subplot(3,1,1)
plot(T(:,6))
xlabel('X')
subplot(3,1,2)
plot(T(:,10))
xlabel('Y')
subplot(3,1,3)
plot(T(:,14))
xlabel('Z')