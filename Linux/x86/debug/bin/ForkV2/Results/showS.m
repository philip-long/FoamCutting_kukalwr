function showS()
clear all,close all,clc
data = load('S');


%set(gca,'ylim',[-0.02,0.02])

%titlename = strcat("deltaS");
figure(1)
title("S");

hold on;
plot(data(:,1),'r');
hold on;
plot(data(:,2),'b');
hold on;
plot(data(:,3),'g');
hold on;
plot(data(:,4),'y');
hold on
plot(data(:,5),'k');
hold on;
plot(data(:,6),'m');
hold on;
plot(data(:,7),'r--');
hold on;
plot(data(:,8),'b--');
hold on;
hold on;
ylim([-500,500])

legend('x1','y1','x2','y2','x3','y3','x4','y4')
