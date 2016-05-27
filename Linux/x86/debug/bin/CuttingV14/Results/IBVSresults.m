% IBVS resuts
clear all,clc
load('Sdelta')
for(i=1:6)
    figure(i)
    plot(Sdelta(:,i))
end
% 
% for i=1:length(Sdelta)
%     plot(i,norm(Sdelta(i,:)))
%     hold on
% end
