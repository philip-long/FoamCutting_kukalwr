function showErrorR()
data = load('MeasuredR');
data2 = load('CommandedR');
figure;

titlename = strcat("Error in Orientation");
title(titlename);
for i=1:3
	for j=1:(length(data))
		Error(j,i)=data(j,i)-data2(j,i);
	end
end

hold on;
plot(Error(:,1),'y');
hold on;
plot(Error(:,2),'m');
hold on;
plot(Error(:,3),'c');
hold on;


legend('X','Y','Z');





