function showall(name)
data = load(name);
dimension = size(data);
num = dimension(2);
figure;

titlename = strcat(name);
title(titlename);
hold on;
for i=1:num
plot(data(:,i));
hold on;
end





