function show3X()
clear all
clc
close all
data = load('MeasuredX');
data2 = load('CommandedX');
figure;

titlename = strcat("Xmeasured V Xdesired");
title(titlename);
plot3(data(:,1),data(:,2),data(:,3),'k');
hold on
plot3(data2(:,1),data2(:,2),data2(:,3),'r');
legend('Xmeasured','Xdesired')
xlabel('X')
ylabel('Y')
zlabel('Z')




