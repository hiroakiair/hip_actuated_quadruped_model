function CoriGrav = myCoriGrav(in1,in2,in3)
%MYCORIGRAV
%    CORIGRAV = MYCORIGRAV(IN1,IN2,IN3)

%    This function was generated by the Symbolic Math Toolbox version 8.3.
%    01-Mar-2023 21:53:33

M = in3(:,1);
dq4 = in2(4,:);
g = in3(:,9);
k_ab = in3(:,7);
l_ab = in3(:,6);
m = in3(:,4);
q3 = in1(3,:);
q4 = in1(4,:);
t2 = cos(q4);
t3 = dq4.^2;
CoriGrav = [l_ab.*m.*t2.*t3;M.*g+g.*m+l_ab.*m.*t3.*sin(q4);k_ab.*(q3-q4);-k_ab.*q3+k_ab.*q4-g.*l_ab.*m.*t2];
