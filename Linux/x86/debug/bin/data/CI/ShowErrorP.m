function showErrorP()
data = load('MeasuredX');
data2 = load('CommandedX');
figure;

titlename = strcat("currentX");
title(titlename);
hold on;
plot(data(:,1)-data2(:,1),'y');
hold on;
plot(data(:,2)-data2(:,2),'m');
hold on;
plot(data(:,3)-data2(:,3),'c');
hold on;


legend('X','Y','Z','4','5','6');




