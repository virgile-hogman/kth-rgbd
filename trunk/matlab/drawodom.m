cd '~/data/kth/set4_4rooms/odom/';
a = dir('odom*')

figure
hold on
for i=1:size(a)
    b = textread(a(i).name);
    plot(b(1),b(2));
end

title('Odometry');
