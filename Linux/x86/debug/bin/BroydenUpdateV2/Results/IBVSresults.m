% IBVS resuts
clear all,clc
Sdelta=load('Sdelta');
Sd=load('Sd');
S=load('S');
for i=1:6
    figure(i)
    plot(Sdelta(:,i))
end



%% Area
Avg=load('MovingAverageForce');
Sd=load('Sd');
S=load('S');
plot(Avg(:,2)*-0.05,'k');
hold on
plot(S(:,3),'r');
plot(Sd(:,3),'b');

figure(2)
plot(Avg(:,2),'k');



% 
% for i=1:length(Sdelta)
%     plot(i,norm(Sdelta(i,:)))
%     hold on
% end





%% 

clear all,close all,clc
data = load('bMt');

Px=data(1:4:end,4);
Py=data(2:4:end,4);
Pz=data(3:4:end,4);

plot(Py)

for i=1:48:length(data)
    uz=data(i:i+2,3)
   plot(i,uz,'r*','MarkerSize',5.0) 
   hold on
   pause(0.1)
end


for i=1:72:length(data)
plot3(Px,Py,Pz)
zlim([0.05 0.12])
data(i:i+3,:)
pause(0.1)
hold on
drawframe(data(i:i+3,:))
end