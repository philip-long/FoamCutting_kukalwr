% Plot orientation of camera

clear bMc Px Py Pz
data = load('bMt');
bMc={}
% change this to camera frame
j=0;
for i=1:4:length(data)
    j=j+1;
    bMc{j}=data(i:i+3,:);
end

for i=1:length(bMc)
    Px(i)=bMc{i}(1,4);
    Py(i)=bMc{i}(2,4);
    Pz(i)=bMc{i}(3,4);
    aAxis(:,i)=bMc{i}(1:3,3);
end
plot(Px,Py)



plot(Py,Pz,'r','Linewidth',2.0)
hold on
legend('Camera Trajectory')
title('3D profile following')
for i=1:10:length(Py)
    quiver(Py(i),Pz(i),aAxis(2,i),aAxis(3,i),0.05)
    pause(0.01)
end

%plot(Px,Py,'r','Linewidth',2.0)
%hold on
%plot(P(1,2:end),P(2,2:end),'Linewidth',2.0)

%% In this slide I must plot the desired poisions
