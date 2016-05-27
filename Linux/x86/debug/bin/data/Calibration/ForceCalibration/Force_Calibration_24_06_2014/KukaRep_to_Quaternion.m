function Q=KukaRep_to_Quaternion(A,B,C)

% This function trasnforms the kuka representation to Quaternion
% Kuka is Rot(A,z)*Rot(B,y)*Rot(C,x)

RA=[cos(A) -sin(A)    0    0
    sin(A)  cos(A)    0    0
    0       0         1    0
    0       0         0    1];

RB= [cos(B)  0      sin(B) 0
    0       1      0      0
    -sin(B) 0      cos(B) 0
    0       0      0      1];

RC=[1        0       0       0
    0        cos(C) -sin(C)  0
    0        sin(C)  cos(C)  0
    0        0       0       1];

R=RA*RB*RC

disp 'Q= w x y z' 
Q=Rot_to_Quaternion(R)

disp ' Qvisp = x y z w'
Qvisp=[Q(2) Q(3) Q(4) Q(1)]
