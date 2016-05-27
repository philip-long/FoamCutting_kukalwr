function show(name,num)
data = load(name);
figure;

titlename = strcat(name," the ", int2str(num), "th value");
title(titlename);
hold on;
plot(data(:,num));




