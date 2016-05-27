% Inverstigations about speed etc
clear all,close all,clc

Curvedata = load('Curve');
bMt = load('bMt');

Px=bMt(1:4:end,4);
Py=bMt(2:4:end,4);
Pz=bMt(3:4:end,4);



plot(Curvedata(:,1),Curvedata(:,3),'r','Linewidth',2.0)
hold on
plot(Px,Py,'b','Linewidth',2.0)
figure(2)
plot(Pz)

%% Velocity base
bV=load('OutputCmdbV');

for i=1:3
figure(i)
plot(bV(:,i),'r');
end


%% Velocity tool Commanded
tV=load('OutputCmdtV');

for i=1:3
figure(i)
plot(tV(:,i),'b');
end


%% Try to plot the actually velocity, versus the commanded velocity
% At least there appears to be a correaltion

bMt = load('bMt');

Px=bMt(1:4:end,4);
Py=bMt(2:4:end,4);
Pz=bMt(3:4:end,4);

bVtcalc=[]
tVtcalc=[]
Cycletime=0.060;
j=1;
for i=2:length(Py)
    bMttemp=bMt(j:j+3,1:4)
    
    bRt=bMttemp(1:3,1:3);
    bVtemp=[Px(i)-Px(i-1), Py(i)-Py(i-1),Pz(i)-Pz(i-1)]./Cycletime
    tVtemp=transpose(bRt)*bVtemp';
    bVtcalc=[bVtcalc;bVtemp];   
    tVtcalc=[tVtcalc;tVtemp'];   
    j=j+4;
end

for i=1:3
    figure(i)
plot(bVtcalc(:,i),'r')
hold on
plot(bV(:,i),'b')
end

for i=1:3
    figure(i)
plot(tVtcalc(:,i),'r')
hold on
plot(tV(:,i),'b')
end