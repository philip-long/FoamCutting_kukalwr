function showdesiredR()
data = load('CommandedR');
figure;

titlename = strcat("desiredR");
title(titlename);
hold on;
plot(data(:,1),'y');
hold on;
plot(data(:,2),'m');
hold on;
plot(data(:,3),'c');
hold on;


legend('1','2','3','4','5','6');




