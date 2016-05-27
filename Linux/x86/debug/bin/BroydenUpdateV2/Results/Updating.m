%% Plot Updating result


clear all,close all,clc
data = load('bMt');
sdelta=load('Sdelta');
s=load('S');
tV=load('OutputCmdtV');
dX=load('OutputdX');
Err=load('ErrorEstim');
dX=load('OutputdX');


Px=data(1:4:end,4);
Py=data(2:4:end,4);
Pz=data(3:4:end,4);

figure(1)
plot(tV)

figure(2)
plot(sdelta)

figure(3)
plot(s)

figure(4)
plot(Err(:,1))
% figure(3)
% plot(dX)

% for i=1:length(dX)
%    plot(i,norm(dX( i,:)),'r*')
%    hold on
% end
