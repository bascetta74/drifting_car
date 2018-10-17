%%%% Optimization problem characterized by cost function + Upper/Lower bounds + Equality and Inequality constraints %%%%
clc
clear all
close all

%--------------------- Test_1 ------------------%
lb1=[0 0]';
ub1=[5 5]';
H1=[2 0;
    0 5];
f1=[-1 3]';
Ain1=[1 -1;
      2 -2];
Bin1=[1 0]';
Aeq1=[ 1 0;
      -1 0];
Beq1=[1 -1]';
[x1,fval1,exitflag1]=cplexqp(H1,f1,Ain1,Bin1,Aeq1,Beq1,lb1,ub1)

%--------------------- Test_2 ------------------%
lb2=[-1 -1 -1]';
ub2=[5 5 5]';
H2=[ 3 0 -1;
     0 2  0;
    -1 0  1];
f2=[-2 3 1]';
Ain2=[ 1 -1 1;
       2 -2 1;
      -1  0 1];
Bin2=[1 0 -1]';
Aeq2=[0 0 -2;
      0 0  3;
      1 0  1];
Beq2=[0 0 1]';
[x2,fval2,exitflag2]=cplexqp(H2,f2,Ain2,Bin2,Aeq2,Beq2,lb2,ub2)

%--------------------- Test_3 ------------------%
lb3=[-1 -1 -1 -1 -1]';
ub3=[5 5 5 5 5]';
H3=[ 4 0  1  0 -1;
     0 5  1  0  0;
     1 1  3 -1  1;
     0 0 -1  2 -1;
    -1 0  1 -1  5];
f3=[1 0 1 2 -1]';
Ain3=[ 1 -1 1 0 -1;
       2 -2 1 0  0;
      -1  0 1 0  1];
Bin3=[1 0 -1]';
Aeq3=[1 -1 0 0 1;
      0 -2 0 1 1;
      1  0 1 0 0];
Beq3=[1 0 -1]';
[x3,fval3,exitflag3]=cplexqp(H3,f3,Ain3,Bin3,Aeq3,Beq3,lb3,ub3)

%--------------------- Test_4 ------------------%
lb4=[-1 -1 -1 -1 -1]';
ub4=[5 5 5 5 5]';
H4=[ 4 0  1  0 -1;
     0 5  1  0  0;
     1 1  3 -1  1;
     0 0 -1  2 -1;
    -1 0  1 -1  5];
f4=[1 0 1 2 -1]';
Ain4=[ 1 -1  1  0 -1;
       2 -2  1  0  0;
      -1  0  1  0  1;
       0  1 -1  2  0;
       1  0  1 -1 -2];
Bin4=[1 0 -1 0 -2]';
Aeq4=[1  0 0 0 1;
      0 -2 0 1 0;
      0  0 0 0 0;
      0  0 0 0 0;
      0  0 0 0 0];
Beq4=[1 0 1 0 0]';
[x4,fval4,exitflag4]=cplexqp(H4,f4,Ain4,Bin4,Aeq4,Beq4,lb4,ub4)
