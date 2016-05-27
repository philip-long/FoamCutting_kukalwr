function showdeltas()
clear all,close all,clc
data = load('deltaS');


%set(gca,'ylim',[-0.02,0.02])

%titlename = strcat("deltaS");
figure(1)
title("Delta S Position error");

hold on;
plot(data(:,1),'y');
hold on;
plot(data(:,2),'m');
hold on;
plot(data(:,3),'c');
hold on;


legend('X','Y','Z')


figure(2)
title("Delta S Orientation Error");
hold on;
plot(data(:,4),'y');
hold on;
plot(data(:,5),'m');
hold on;
plot(data(:,6),'c');
hold on;
legend('ut(1)','ut(2)','ut(3)');

