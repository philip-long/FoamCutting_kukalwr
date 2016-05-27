% This file Generates W, the matrix that relates
% the unknown parameters to the force felt at the
% TCP. In theory this file should remain untouched
% since all variables are symbolic
%
%
% W will be used in both GenerateX.m and void ResolveForceatTcP in
% FunctionLibrary.cpp
%  Philip Long 2013
%

% Force sensor to base
%%  LETS TRY IT AGAIN SOMETHING SOMEWHERE MUST BE WRONG RESOLUTION ASSTEMPT 2.
% Cutting Logbook 29-05-2013
%
% Symbolic Variables
clear all,clc
% Force sensor to base
syms fMb11 fMb12 fMb13 fMb21 fMb22 fMb23 fMb31 fMb32 fMb33
syms tMb11 tMb12 tMb13 tMb21 tMb22 tMb23 tMb31 tMb32 tMb33
syms tMf11 tMf12 tMf13 tMf21 tMf22 tMf23 tMf31 tMf32 tMf33
syms fMt11 fMt12 fMt13 fMt21 fMt22 fMt23 fMt31 fMt32 fMt33 fMt14 fMt24 fMt34
syms b1 b2 b3 b4 b5 b6 fPxt fPyt fPzt M_fs M_t

% Distance from Centre of mass of tool to force sensor
syms cgPXt cgPYt cgPZt

% 


Bias=[b1; b2; b3; b4; b5; b6];


fsMb=[fMb11 fMb12 fMb13
      fMb21 fMb22 fMb23
     fMb31 fMb32 fMb33];
 
tMf=[tMf11 tMf12 tMf13
      tMf21 tMf22 tMf23
     tMf31 tMf32 tMf33];
 
 
 fMt=[fMt11 fMt12 fMt13 fMt14
      fMt21 fMt22 fMt23 fMt24
     fMt31 fMt32 fMt33  fMt34];
 
 tMb=[tMb11 tMb12 tMb13
      tMb21 tMb22 tMb23
     tMb31 tMb32 tMb33];
 
cgPt=[cgPXt cgPYt cgPZt];

 % Force due to mass of tool in base frame
bMasst_cg=[0;0;-9.81*M_t;0;0;0];
% Force due to mass of sensor in base frame
bMassfs=[0;0;-9.81*M_fs;0;0;0];  
 

% 1. Put Massfs in forceSensorFrame add Bias then
% 1.1 Change frame
fsRb=[ fsMb(1:3,1:3) zeros(3)
       zeros(3) fsMb(1:3,1:3)];

fsMassfs=fsRb*bMassfs;
% 1.2 Add Bias
fsMassfsBias=fsMassfs+Bias;

%1.3 Change to Tool Tip

tRfs=tMf(1:3,1:3);
fsPt=fMt(1:3,4);

tTfs=[tRfs              zeros(3)
      -tRfs*skew(fsPt)  tRfs];


tMassfsBias=tTfs*fsMassfsBias;

% 2. Put bMasst in tool frame
% 2.1 Change from base to Cg frame
tRb=[ tMb(1:3,1:3) zeros(3)
       zeros(3) tMb(1:3,1:3)];

 tMasst_cg=tRb*bMasst_cg;
 
 % 2.2 Screw Tranform
 
 tTcg=[eye(3)             zeros(3)
      -skew(cgPt)  eye(3)];
 
 tMasst_t=tTcg*tMasst_cg;
 
 % 3 Add terms together
 
 Resolved=tMasst_t+tMassfsBias;
 %% Here the mass times centre of mass is replaced by one variable, I did it in noteped for shame.....
 syms M_tcgPXt M_tcgPYt M_tcgPZt
 
 Resolved =[
                                                                                                                                      tMf11*(b1 - (981*M_fs*fMb13)/100) - (981*M_t*tMb13)/100 + tMf12*(b2 - (981*M_fs*fMb23)/100) + tMf13*(b3 - (981*M_fs*fMb33)/100)
                                                                                                                                      tMf21*(b1 - (981*M_fs*fMb13)/100) - (981*M_t*tMb23)/100 + tMf22*(b2 - (981*M_fs*fMb23)/100) + tMf23*(b3 - (981*M_fs*fMb33)/100)
                                                                                                                                      tMf31*(b1 - (981*M_fs*fMb13)/100) - (981*M_t*tMb33)/100 + tMf32*(b2 - (981*M_fs*fMb23)/100) + tMf33*(b3 - (981*M_fs*fMb33)/100)
 (fMt14*tMf12 - fMt24*tMf11)*(b3 - (981*M_fs*fMb33)/100) - (fMt14*tMf13 - fMt34*tMf11)*(b2 - (981*M_fs*fMb23)/100) + (fMt24*tMf13 - fMt34*tMf12)*(b1 - (981*M_fs*fMb13)/100) + b4*tMf11 + b5*tMf12 + b6*tMf13 - (981*M_tcgPZt*tMb23)/100 + (981*M_tcgPYt*tMb33)/100
 (fMt14*tMf22 - fMt24*tMf21)*(b3 - (981*M_fs*fMb33)/100) - (fMt14*tMf23 - fMt34*tMf21)*(b2 - (981*M_fs*fMb23)/100) + (fMt24*tMf23 - fMt34*tMf22)*(b1 - (981*M_fs*fMb13)/100) + b4*tMf21 + b5*tMf22 + b6*tMf23 + (981*M_tcgPZt*tMb13)/100 - (981*M_tcgPXt*tMb33)/100
 (fMt14*tMf32 - fMt24*tMf31)*(b3 - (981*M_fs*fMb33)/100) - (fMt14*tMf33 - fMt34*tMf31)*(b2 - (981*M_fs*fMb23)/100) + (fMt24*tMf33 - fMt34*tMf32)*(b1 - (981*M_fs*fMb13)/100) + b4*tMf31 + b5*tMf32 + b6*tMf33 - (981*M_tcgPYt*tMb13)/100 + (981*M_tcgPXt*tMb23)/100];
 
 
 
 
 
 %%
 
 X=[b1 b2 b3 b4 b5 b6 M_fs M_tcgPXt M_tcgPYt M_tcgPZt M_t]
 W=jacobian(Resolved,X)
 
 
 %