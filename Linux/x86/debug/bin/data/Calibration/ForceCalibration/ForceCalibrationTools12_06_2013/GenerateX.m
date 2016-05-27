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
Force = importdata('Force.txt');

% The tool data 
bMt_new = importdata('bMe.txt');

j=1;
for i=1:4:(length(bMt_new))
    bMt(1:4,:,j)=bMt_new(i:i+3,:);
    j=j+1;
end

%X =[ b1, b2, b3, b4, b5, b6, M_fs, fPXcg_t, fPYcg_t, fPZcg_t]
 
% This is taken from observation
% eMf=[0.707 0.707 0 0
%      -0.707  0.707 0 0
%      0   0 1 0.048
%       0 0 0 1];
%     
% eMt=[1 0 0 -0.01635
%      0 1 0 0.008
%      0 0 1 0.3337
%      0 0 0 1];
 eMf=[ 1   1 0 0
       0   1  0 0
       0   0 1 0.058
       0 0 0 1];
 eMt=[1 0 0 -0.007135
      0 1 0 0.014077
      0 0 1 0.3338
      0 0 0 1];
 
 tMf=eMt\eMf;
 fMt=eMf\eMt;
% -----------------------------ASIGN THE VARIABLES-------------------------


tMf11=tMf(1,1);tMf12=tMf(1,2);tMf13=tMf(1,3);
tMf21=tMf(2,1);tMf22=tMf(2,2);tMf23=tMf(2,3);
tMf31=tMf(3,1);tMf32=tMf(3,2);tMf33=tMf(3,3);


M_t=1;
fPxt=fMt(1,4);
fPyt=fMt(2,4);
fPzt=fMt(3,4);
LD=skew([fPxt;fPyt;fPzt])*tMf(1:3,1:3);

Y_Measure=[];
W_observe=[];
i=1
for i=1:length(Force)
    
    % Find variables
    % Find measured force in newtons and newton metres
    F=Force(i,:)/1000000;
    %fMb=inv(bMe(:,:,1)*eMf);
    
    fMb=inv(bMt(:,:,i)*tMf);
    
    fMb11=fMb(1,1);fMb12=fMb(1,2);fMb13=fMb(1,3);
    fMb21=fMb(2,1);fMb22=fMb(2,2);fMb23=fMb(2,3);
    fMb31=fMb(3,1);fMb32=fMb(3,2);fMb33=fMb(3,3);


    

    % Transform measured force to tool frame
    tF=[tMf(1:3,1:3) zeros(3);LD tMf(1:3,1:3)]*F';
    
    
    % W is found from Resolution Procedure
W =[     -tMf11,                  -tMf12,                  -tMf13,      0,      0,      0, 0,                                                     0,                                                     0,                                                     0
         -tMf21,                  -tMf22,                  -tMf23,      0,      0,      0, 0,                                                     0,                                                     0,                                                     0
                  -tMf31,                  -tMf32,                  -tMf33,      0,      0,      0, 0,                                                     0,                                                     0,                                                     0
 fPzt*tMf21 - fPyt*tMf31, fPzt*tMf22 - fPyt*tMf32, fPzt*tMf23 - fPyt*tMf33, -tMf11, -tMf12, -tMf13, 0, (981*M_t*fMb33*tMf12)/100 - (981*M_t*fMb23*tMf13)/100, (981*M_t*fMb13*tMf13)/100 - (981*M_t*fMb33*tMf11)/100, (981*M_t*fMb23*tMf11)/100 - (981*M_t*fMb13*tMf12)/100
 fPxt*tMf31 - fPzt*tMf11, fPxt*tMf32 - fPzt*tMf12, fPxt*tMf33 - fPzt*tMf13, -tMf21, -tMf22, -tMf23, 0, (981*M_t*fMb33*tMf22)/100 - (981*M_t*fMb23*tMf23)/100, (981*M_t*fMb13*tMf23)/100 - (981*M_t*fMb33*tMf21)/100, (981*M_t*fMb23*tMf21)/100 - (981*M_t*fMb13*tMf22)/100
 fPyt*tMf11 - fPxt*tMf21, fPyt*tMf12 - fPxt*tMf22, fPyt*tMf13 - fPxt*tMf23, -tMf31, -tMf32, -tMf33, 0, (981*M_t*fMb33*tMf32)/100 - (981*M_t*fMb23*tMf33)/100, (981*M_t*fMb13*tMf33)/100 - (981*M_t*fMb33*tMf31)/100, (981*M_t*fMb23*tMf31)/100 - (981*M_t*fMb13*tMf32)/100];
 
    Y_Measure=[Y_Measure; tF];
    W_observe=[W_observe;W];
end

x=pinv(W_observe)*Y_Measure