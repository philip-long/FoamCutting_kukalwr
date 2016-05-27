function showforce()
clear all
data = load('MeasuredForce');

data2 = load('EstimatedForce');
figure(3)
title("Force");


plot(data(:,1),'y');
hold on;
plot(data(:,2),'m');
plot(data(:,3),'c');
plot(data2(:,1),'r');
plot(data2(:,2),'b');
plot(data2(:,3),'k');

legend('Fx','Fy','Fz','Fx_Kuka','Fy_Kuka','Fz_Kuka')


figure(4)
title("Moments");

hold on;
plot(data(:,4),'y');
plot(data(:,5),'m');
plot(data(:,6),'c');
plot(data2(:,4),'r');
plot(data2(:,5),'b');
plot(data2(:,6),'k');
legend('taux','tauy','tauz','taux_Kuka','tauy_Kuka','tauz_Kuka');

