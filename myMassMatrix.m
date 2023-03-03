function M_mat = myMassMatrix(in1,in2)
%MYMASSMATRIX
%    M_MAT = MYMASSMATRIX(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.3.
%    03-Mar-2023 21:14:17

J_1 = in2(:,2);
J_2 = in2(:,5);
M = in2(:,1);
l_ab = in2(:,6);
m = in2(:,4);
q4 = in1(4,:);
t2 = cos(q4);
t3 = sin(q4);
t4 = M+m;
t5 = l_ab.*m.*t2;
t6 = l_ab.*m.*t3;
t7 = -t5;
M_mat = reshape([t4,0.0,0.0,t6,0.0,t4,0.0,t7,0.0,0.0,J_1,0.0,t6,t7,0.0,J_2+l_ab.^2.*m],[4,4]);
