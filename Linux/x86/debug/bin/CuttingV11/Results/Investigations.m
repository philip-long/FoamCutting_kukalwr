% Inverstigations about speed etc
clear all,close all,clc
CurvePoints=load('DataOut');
bMt = load('bMt');
X=CurvePoints(1:4:end,4);
Y=CurvePoints(2:4:end,4);
Px=bMt(1:4:end,4);
Py=bMt(2:4:end,4);
Pz=bMt(3:4:end,4);


plot(X,Y,'g:','Linewidth',2.0)
hold on
plot(Px,Py,'b','Linewidth',2.0)
plot(Pz)

%% Velocity base
bV=load('OutputCmdbV');

for i=1:3
figure(i)
plot(bV(:,i),'r');
end


%% Velocity tool
tV=load('OutputCmdtV');

for i=1:3
figure(i)
plot(tV(:,i),'b');
end

%% Velocity scaled
tVs=load('OutputCmdtVsca');

for i=1:3
figure(i)
plot(tVs(:,i),'b');
end