function showq()
data = load('torqueJ');
figure;

titlename = strcat("torqueJ");
title(titlename);
hold on;
plot(data(:,1),'y');
hold on;
plot(data(:,2),'m');
hold on;
plot(data(:,3),'c');
hold on;
plot(data(:,4),'r');
hold on;
plot(data(:,5),'g');
hold on;
plot(data(:,6),'b');
hold on;
plot(data(:,7),'k');
hold on;
legend('1','2','3','4','5','6','7');



