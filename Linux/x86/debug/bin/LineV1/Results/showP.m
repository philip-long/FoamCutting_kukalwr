function showP()
clear all,close all,clc
data = load('bMt');


%set(gca,'ylim',[-0.02,0.02])

%titlename = strcat("deltaS");
figure(1)
title("Delta x,y,L");

hold on;
plot(data(1:4:end,4),'y');
hold on;
plot(data(2:4:end,4),'m');
hold on;
plot(data(3:4:end,4),'c');
hold on;

legend('Px','Py','Pz')



