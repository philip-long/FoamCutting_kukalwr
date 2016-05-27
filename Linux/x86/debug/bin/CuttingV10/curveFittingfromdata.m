%
clear all clc
bMt=load('DataOut')
Px=[bMt(1:4:end,4)];
Py=[bMt(2:4:end,4)];

P=polyfit(Px,Py,2)
plot(Px,Py,'r*')
hold on
plot(Px,polyval(P,Px),'b')
polyval(P,0)