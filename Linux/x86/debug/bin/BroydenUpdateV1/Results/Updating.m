%% Plot Updating result

clear all,close all,clc
data = load('bMt');
sdelta=load('Sdelta');
s=load('S');
cVc=load('OutputCmdcVc');
dX=load('OutputdX');

Err=load('ErrorEstim');
dX=load('OutputdX');
dX2=load('OutputCmdtV');

dxFil=load('OutputMovingAverageDx');
dsFil=load('OutputMovingAverageDs');
ErrFil=load('MovingAverageError');
%%
Px=data(1:4:end,4);
Py=data(2:4:end,4);
Pz=data(3:4:end,4);
plot(ErrFil)
figure(2)
plot(sdelta)

figure(3)
plot(s)

figure(4)
plot(Err)
% figure(3)
% plot(dX)

% for i=1:length(dX)
%    plot(i,norm(dX( i,:)),'r*')
%    hold on
% end



%%  Checking dx versus Camera velocity
cVc=load('OutputCmdcVc');
dX=load('OutputdX');

for i=1:6
plot(cVc(:,2)*0.06*0.01,'b--')
plot(dxFil(:,i),'b--')
hold on
plot(dX(:,i),'r--')
i
pause()
hold off 
end



for i=1:6
plot(cVc(:,i)*0.06*0.005,'r--')
hold on
plot(dxFil(:,i),'b--')
i
pause()
hold off 
end

for i=1:length(ErrFil)
    plot(i,norm(ErrFil(i,:)),'r*')
    hold on
end
    