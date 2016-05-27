
eMt=[   1.000    0.000    0.000   -0.007
   0.000    1.000    0.000    0.014
   0.000    0.000    1.000    0.334
   0.000    0.000    0.000    1.000];

eMf=[     1.000    0.000    0.000    0.000
   0.000    1.000    0.000    0.000
   0.000    0.000    1.000    0.058
   0.000    0.000    0.000    1.000];

tMb=[   0.618    0.009    0.786   -0.539
  -0.011    1.000   -0.003    0.016
  -0.786   -0.006    0.618   -1.274
   0.000    0.000    0.000    1.000]



fMb=[   0.618    0.009    0.786   -0.546
  -0.011    1.000   -0.003    0.030
  -0.786   -0.006    0.618   -0.998
   0.000    0.000    0.000    1.000];

 tMf=eMt\eMf;
 fMt=eMf\eMt;
    
 bMt=inv(tMb);
 fMb=inv(bMt*tMf);
 
 
tMf11=tMf(1,1);tMf12=tMf(1,2);tMf13=tMf(1,3);
tMf21=tMf(2,1);tMf22=tMf(2,2);tMf23=tMf(2,3);
tMf31=tMf(3,1);tMf32=tMf(3,2);tMf33=tMf(3,3);

fMt11=fMt(1,1);fMt12=fMt(1,2);fMt13=fMt(1,3);fMt14=fMt(1,4);
fMt21=fMt(2,1);fMt22=fMt(2,2);fMt23=fMt(2,3);fMt24=fMt(2,4);
fMt31=fMt(3,1);fMt32=fMt(3,2);fMt33=fMt(3,3);fMt34=fMt(3,4);

fPxt=fMt(1,4);
fPyt=fMt(2,4);
fPzt=fMt(3,4);
LD=skew([fPxt;fPyt;fPzt])*tMf(1:3,1:3)

    fMb11=fMb(1,1);fMb12=fMb(1,2);fMb13=fMb(1,3);
    fMb21=fMb(2,1);fMb22=fMb(2,2);fMb23=fMb(2,3);
    fMb31=fMb(3,1);fMb32=fMb(3,2);fMb33=fMb(3,3);

    tMb11=tMb(1,1);tMb12=tMb(1,2);tMb13=tMb(1,3);
    tMb21=tMb(2,1);tMb22=tMb(2,2);tMb23=tMb(2,3);
    tMb31=tMb(3,1);tMb32=tMb(3,2);tMb33=tMb(3,3);
    

    
    fMb11=fMb(1,1);fMb12=fMb(1,2);fMb13=fMb(1,3);
    fMb21=fMb(2,1);fMb22=fMb(2,2);fMb23=fMb(2,3);
    fMb31=fMb(3,1);fMb32=fMb(3,2);fMb33=fMb(3,3);

    tMb11=tMb(1,1);tMb12=tMb(1,2);tMb13=tMb(1,3);
    tMb21=tMb(2,1);tMb22=tMb(2,2);tMb23=tMb(2,3);
    tMb31=tMb(3,1);tMb32=tMb(3,2);tMb33=tMb(3,3);
    
x =[
       15.332
      -12.157
      -1.4856
        5.441
       6.3645
     -0.21071
       0.2127
     0.010534
    -0.014079
     -0.15996
      0.25709]  ;
%


Force=[12556006.000,-11238571.000,-3884054.000,-1161229.000,-2044808.000,58813.000];


 F=Force/1000000;



    % W is found from Resolution Procedure
W =[                     tMf11,                     tMf12,                     tMf13,     0,     0,     0,                                                                 - (981*fMb13*tMf11)/100 - (981*fMb23*tMf12)/100 - (981*fMb33*tMf13)/100,                0,                0,                0, -(981*tMb13)/100
                     tMf21,                     tMf22,                     tMf23,     0,     0,     0,                                                                 - (981*fMb13*tMf21)/100 - (981*fMb23*tMf22)/100 - (981*fMb33*tMf23)/100,                0,                0,                0, -(981*tMb23)/100
                     tMf31,                     tMf32,                     tMf33,     0,     0,     0,                                                                 - (981*fMb13*tMf31)/100 - (981*fMb23*tMf32)/100 - (981*fMb33*tMf33)/100,                0,                0,                0, -(981*tMb33)/100
 fMt24*tMf13 - fMt34*tMf12, fMt34*tMf11 - fMt14*tMf13, fMt14*tMf12 - fMt24*tMf11, tMf11, tMf12, tMf13, (981*fMb23*(fMt14*tMf13 - fMt34*tMf11))/100 - (981*fMb33*(fMt14*tMf12 - fMt24*tMf11))/100 - (981*fMb13*(fMt24*tMf13 - fMt34*tMf12))/100,                0,  (981*tMb33)/100, -(981*tMb23)/100,                0
 fMt24*tMf23 - fMt34*tMf22, fMt34*tMf21 - fMt14*tMf23, fMt14*tMf22 - fMt24*tMf21, tMf21, tMf22, tMf23, (981*fMb23*(fMt14*tMf23 - fMt34*tMf21))/100 - (981*fMb33*(fMt14*tMf22 - fMt24*tMf21))/100 - (981*fMb13*(fMt24*tMf23 - fMt34*tMf22))/100, -(981*tMb33)/100,                0,  (981*tMb13)/100,                0
 fMt24*tMf33 - fMt34*tMf32, fMt34*tMf31 - fMt14*tMf33, fMt14*tMf32 - fMt24*tMf31, tMf31, tMf32, tMf33, (981*fMb23*(fMt14*tMf33 - fMt34*tMf31))/100 - (981*fMb33*(fMt14*tMf32 - fMt24*tMf31))/100 - (981*fMb13*(fMt24*tMf33 - fMt34*tMf32))/100,  (981*tMb23)/100, -(981*tMb13)/100,                0,                0]

[tMf(1:3,1:3) zeros(3);LD tMf(1:3,1:3)]
 tF=[tMf(1:3,1:3) zeros(3);LD tMf(1:3,1:3)]*F'
 
 
 EstimatedForces=W*x;

SensedForce=tF'
ResolvedForce= (tF-EstimatedForces)'
