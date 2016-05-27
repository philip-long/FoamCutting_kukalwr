%
% Orientation Error
E=load('Uv');
% RXYZ
XYZ=load('OutputRXYZ')
plot(XYZ(:,1)*180/pi,'r')
hold on
plot(XYZ(:,2)*180/pi,'g')
plot(XYZ(:,3)*180/pi,'b')


%  Image moments

Sd=load('Sd');
S=load('S');