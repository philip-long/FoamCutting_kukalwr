function showforce()
clear all
data = load('MeasuredForce');

figure(3)
title("Force");

hold on;
plot(data(:,1),'y');
hold on;
plot(data(:,2),'m');
hold on;
plot(data(:,3),'c');
hold on;


legend('Fx','Fy','Fz')


figure(4)
title("Moments");

hold on;
plot(data(:,4),'y');
hold on;
plot(data(:,5),'m');
hold on;
plot(data(:,6),'c');
hold on;
legend('taux','tauy','tauz');

