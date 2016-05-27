function showcurrentR()
data = load('MeasuredR');
figure;

titlename = strcat("currentR");
title(titlename);
hold on;
plot(data(:,1),'y');
hold on;
plot(data(:,2),'m');
hold on;
plot(data(:,3),'c');
hold on;


legend('1','2','3','4','5','6');




