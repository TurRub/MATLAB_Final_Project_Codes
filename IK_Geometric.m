clear all 
syms L1 L2 theta1 theta2 D XE YE %where D4 = D + 0.5 according to our schematic

XE_RHS = 0.5*cos(theta1) + (0.5+D)*cos(theta1+theta2);
YE_RHS = 0.3+ 0.5*sin(theta1) + (0.5+D)*sin(theta1+theta2);

XE_EQ = XE == XE_RHS;
YE_EQ = YE == YE_RHS;

S = solve([XE_EQ YE_EQ], [theta1 theta2 D])
simplify(S.theta1)
simplify(S.theta2)
simplify(S.D)
