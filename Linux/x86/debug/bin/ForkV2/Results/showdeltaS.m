function showdeltaS()
%clear all
%close all
%clc
data = load('deltaS');


%set(gca,'ylim',[-0.02,0.02])

%titlename = strcat("deltaS");
figure(1)
title("deltas");

%hold on;
plot(data(:,1),'r');
hold on;
plot(data(:,2),'b');
hold on;
plot(data(:,3),'m');
hold on;
plot(data(:,4),'c');
hold on
plot(data(:,5),'y');
hold on;
plot(data(:,6),'k');
hold on;
plot(data(:,7),'r--');
hold on;
plot(data(:,8),'b--');
hold on;
%ylim([-500,500])

legend('x1','y1','x2','y2','x3','y3','x4','y4')

