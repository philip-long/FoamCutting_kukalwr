clear all,clc
% This File generates the vector X of unknown
% parameters using W from GenerateW. 
%
% We would like the sensor to give 0 for forces when the tool is not in contact with the world.
%
% Therefore we solve for x for many configuration
% such that the Measured force-Estimated force
% gives zero. 
%
% x contains the bias the mass of the force sensor
% and the mass of the tool. In theory any forces
% felt by the tool when not in contact and in
% static conditions should be due to x
%
% 
% 
% 
%
% This function is intended to be used with
% GenerateW.m and void ResolveForceatTcP in
% FunctionLibrary.cpp
%  Philip Long 2013
%
% The force data at the force data frame
Force = importdata('Force');

% The tool data 
bMt_new = importdata('bMt');

eMt= importdata('eMt');

eMf=importdata('eMf');

j=1;
for i=1:4:(length(bMt_new))
    bMt(1:4,:,j)=bMt_new(i:i+3,:);
    j=j+1;
end


% eMt=[0.7071    0.7071         0  0.019589
%      -0.7071    0.7071         0  -0.02236
%       0 0 1 0.3338
%       0 0 0 1];  
%   
%   
%   eMt=[0.7071    0.7071         0  0.019589
%      -0.7071    0.7071         0  -0.02236
%       0 0 1 0.3338
%       0 0 0 1];  

% eMt=[1.0     0.0     0.0      0.019589
%      0.0     0.70711 0.70711  -0.02236
%      0.0    -0.70711 0.70711   0.377
%      0.0     0.0     0.0       1.0000 ]
  
  
  
 tMf=eMt\eMf;
 fMt=eMf\eMt;
% -----------------------------ASIGN THE VARIABLES-------------------------


tMf11=tMf(1,1);tMf12=tMf(1,2);tMf13=tMf(1,3);tMf14=tMf(1,4);
tMf21=tMf(2,1);tMf22=tMf(2,2);tMf23=tMf(2,3);tMf24=tMf(2,4);
tMf31=tMf(3,1);tMf32=tMf(3,2);tMf33=tMf(3,3);tMf34=tMf(3,4);


fMt11=fMt(1,1);fMt12=fMt(1,2);fMt13=fMt(1,3);fMt14=fMt(1,4);
fMt21=fMt(2,1);fMt22=fMt(2,2);fMt23=fMt(2,3);fMt24=fMt(2,4);
fMt31=fMt(3,1);fMt32=fMt(3,2);fMt33=fMt(3,3);fMt34=fMt(3,4);



fPxt=fMt(1,4);
fPyt=fMt(2,4);
fPzt=fMt(3,4);
LD=skew([fPxt;fPyt;fPzt])*tMf(1:3,1:3);

Y_Measure=[];
W_observe=[];

for i=1:length(Force)
    
    % Find variables
    % Find measured force in newtons and newton metres
    F=Force(i,:)/1000000;
    %fMb=inv(bMe(:,:,1)*eMf);
    tMb=inv(bMt(:,:,i));
    fMb=inv(bMt(:,:,i)*tMf);
    
    
    fMb11=fMb(1,1);fMb12=fMb(1,2);fMb13=fMb(1,3);
    fMb21=fMb(2,1);fMb22=fMb(2,2);fMb23=fMb(2,3);
    fMb31=fMb(3,1);fMb32=fMb(3,2);fMb33=fMb(3,3);
    
    
    tMb11=tMb(1,1);tMb12=tMb(1,2);tMb13=tMb(1,3);
    tMb21=tMb(2,1);tMb22=tMb(2,2);tMb23=tMb(2,3);
    tMb31=tMb(3,1);tMb32=tMb(3,2);tMb33=tMb(3,3);

    

    % Transform measured force to tool frame
    tF=[tMf(1:3,1:3) zeros(3);LD tMf(1:3,1:3)]*F';
    
    
    % W is found from Resolution Procedure
W =[                     tMf11,                     tMf12,                     tMf13,     0,     0,     0,                                                                 - (981*fMb13*tMf11)/100 - (981*fMb23*tMf12)/100 - (981*fMb33*tMf13)/100,                0,                0,                0, -(981*tMb13)/100
                     tMf21,                     tMf22,                     tMf23,     0,     0,     0,                                                                 - (981*fMb13*tMf21)/100 - (981*fMb23*tMf22)/100 - (981*fMb33*tMf23)/100,                0,                0,                0, -(981*tMb23)/100
                     tMf31,                     tMf32,                     tMf33,     0,     0,     0,                                                                 - (981*fMb13*tMf31)/100 - (981*fMb23*tMf32)/100 - (981*fMb33*tMf33)/100,                0,                0,                0, -(981*tMb33)/100
 fMt24*tMf13 - fMt34*tMf12, fMt34*tMf11 - fMt14*tMf13, fMt14*tMf12 - fMt24*tMf11, tMf11, tMf12, tMf13, (981*fMb23*(fMt14*tMf13 - fMt34*tMf11))/100 - (981*fMb33*(fMt14*tMf12 - fMt24*tMf11))/100 - (981*fMb13*(fMt24*tMf13 - fMt34*tMf12))/100,                0,  (981*tMb33)/100, -(981*tMb23)/100,                0
 fMt24*tMf23 - fMt34*tMf22, fMt34*tMf21 - fMt14*tMf23, fMt14*tMf22 - fMt24*tMf21, tMf21, tMf22, tMf23, (981*fMb23*(fMt14*tMf23 - fMt34*tMf21))/100 - (981*fMb33*(fMt14*tMf22 - fMt24*tMf21))/100 - (981*fMb13*(fMt24*tMf23 - fMt34*tMf22))/100, -(981*tMb33)/100,                0,  (981*tMb13)/100,                0
 fMt24*tMf33 - fMt34*tMf32, fMt34*tMf31 - fMt14*tMf33, fMt14*tMf32 - fMt24*tMf31, tMf31, tMf32, tMf33, (981*fMb23*(fMt14*tMf33 - fMt34*tMf31))/100 - (981*fMb33*(fMt14*tMf32 - fMt24*tMf31))/100 - (981*fMb13*(fMt24*tMf33 - fMt34*tMf32))/100,  (981*tMb23)/100, -(981*tMb13)/100,                0,                0];

    Y_Measure=[Y_Measure; tF];
    W_observe=[W_observe;W];
end

x=pinv(W_observe)*Y_Measure



file1=fopen('IdentifiedForceParameter','a+');
fprintf(file1,'\n');
for i=1:length(x)
fprintf(file1,num2str(x(i)));
fprintf(file1,';');
end
fprintf(file1,'\n');
fclose(file1);