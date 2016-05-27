%
clear all,close all, clc
bMt=load('DataOut')
Px=[bMt(1:4:end,4)];
Py=[bMt(2:4:end,4)];
Py=Py+ones(length(Py),1)*0.05;

P=polyfit(Px,Py,2)
plot(Px,Py,'r*')
hold on
plot(Px,polyval(P,Px),'b')

fprintf('p[0]=%f ; p[1]=%f ; p[2]=%f;\n',P(3),P(2),P(1));

%p[0]=1.238428 ; p[1]=3.579001 ; p[2]=3.229153;
%p[0]=1.288428 ; p[1]=3.579001 ; p[2]=3.229153;