function showcurrentX()
data = load('MeasuredX');
figure;

titlename = strcat("currentX");
title(titlename);
hold on;
plot(data(:,1),'y');
hold on;
plot(data(:,2),'m');
hold on;
plot(data(:,3),'c');
hold on;






