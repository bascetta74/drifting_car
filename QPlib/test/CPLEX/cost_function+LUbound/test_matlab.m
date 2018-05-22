%%%% Optimization problem characterized by cost function + Upper/Lower bounds %%%%
clc
clear all
close all

%--------------------- Test------------------%
lb1=[0 0];
ub1=[5 5];
H1=[2 0;
    0 5];
f1=[-1 3];
[x1,fval1,exitflag1]=cplexqp(H1,f1,[],[],[],[],lb1,ub1)


%--------------------- Test_1 ------------------%
lb2=[-1 -1 -1];
ub2=[5 5 5];
H2=[ 3 0 -1;
     0 2  0;
    -1 0  1];
f2=[-2 3 1];
[x2,fval2,exitflag2]=cplexqp(H2,f2,[],[],[],[],lb2,ub2)

%--------------------- Test_2 ------------------%
lb3=[-1 -1 -1 -1 -1];
ub3=[5 5 5 5 5];
H3=[ 4 0  1  0 -1;
     0 5  1  0  0;
     1 1  3 -1  1;
     0 0 -1  2 -1;
    -1 0  1 -1  5];
f3=[1 0 1 2 -1];
[x3,fval3,exitflag3]=cplexqp(H3,f3,[],[],[],[],lb3,ub3)
