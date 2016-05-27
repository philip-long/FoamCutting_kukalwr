% This script is to preform camera calibration and see if my results are
% nonsense or good. In theory the camera frame should be aligned on the
% Z-axis. This is a secondary scrip to give me the camera to 
% End effector as I am changing tools quite frequently
clear all
clc
importfiles('cMo.txt')
importfiles('bMe.txt')


% Step 1. get the inverse of cMo and reaarange the matrices
oMc=zeros(4,4,length(cMo)/4);
bMe_new=zeros(4,4,length(bMe)/4);

j=1;
for i=1:4:(length(cMo))
    oMc(1:4,:,j)=inv(cMo(i:i+3,:));
    bMe_new(1:4,:,j)=bMe(i:i+3,:);
    j=j+1;
end

% Step 2 eliminate all redundant data
counter=1
indexer=[]
Cam1=oMc;
Tool1=bMe_new;
Tolerance=0.005
for i=2:(length(oMc))
 
    
    if max( CartesianError([oMc(1:4,:,i);oMc(1:4,:,i-1)]))>Tolerance

         oMc(1:4,:,i);
         oMc(1:4,:,i-1);
         counter=counter+1 ; 
    else
        indexer=[indexer i];
    end
 
end
         Cam1(:,:,indexer)=[];
         Tool1(:,:,indexer)=[];


eHc =handEye(Tool1,Cam1)

file1=fopen('eHc','a+');


fprintf(file1,'\n');

for i=1:4
    for j=1:4
    fprintf(file1,num2str(eHc(i,j)));
    fprintf(file1,'\t');
    end
fprintf(file1,'\n');
end
fprintf(file1,'\n');
