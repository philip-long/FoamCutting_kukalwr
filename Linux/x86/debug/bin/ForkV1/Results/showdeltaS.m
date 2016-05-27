function showS()
clear all,close all,clc
data = load('deltaS');


%set(gca,'ylim',[-0.02,0.02])

%titlename = strcat("deltaS");
figure(1)
title("Delta x,y,L");

hold on;
plot(data(:,1),'y');
hold on;
plot(data(:,2),'m');
hold on;
plot(data(:,3),'c');
hold on;
ylim([-500,500])

legend('Xp','Yp','L')


figure(2)
title("theta error");
hold on;
plot(data(:,4),'y');
hold on;
legend('theta');
ylim([-pi,pi])
