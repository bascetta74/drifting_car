%%%% Optimization problem characterized by only cost function %%%%
clc
clear all
close all

%--------------------- Test_1 ------------------%

H1=[2 0;
    0 5];
f1=[-1 3];
[x1,fval1,exitflag1]=cplexqp(H1,f1,[],[],[],[],[],[])


%--------------------- Test_2 ------------------%

H2=[ 3 0 -1;
     0 2  0;
    -1 0  1];
f2=[-2 3 1];
[x2,fval2,exitflag2]=cplexqp(H2,f2,[],[],[],[],[],[])

%--------------------- Test_3 ------------------%

H3=[ 4 0  1  0 -1;
     0 5  1  0  0;
     1 1  3 -1  1;
     0 0 -1  2 -1;
    -1 0  1 -1  5];
f3=[1 0 1 2 -1];
[x3,fval3,exitflag3]=cplexqp(H3,f3,[],[],[],[],[],[])



