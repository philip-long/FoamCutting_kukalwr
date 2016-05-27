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
%ylim([-500,500])

legend('x','y','z','uxt','uyt','uzt')
