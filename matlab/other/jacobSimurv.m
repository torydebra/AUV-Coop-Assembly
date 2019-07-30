
eta1 = [      -0.563109;
-0.101872;
  7.77818


];
eta2 = rotm2eul([           0.999989   -0.0042577  -0.00196991;
  0.00425769     0.999991 -9.03251e-06;
  0.00196993  6.45163e-07     0.999998;
], ...
    'XYZ');
eta_ee1 = [       0.884727;
-0.0826805;
    9.5709

];

%n = 4;
r_B0_B = [0.000; 0.000; 0.950];
R_0_B  = [-1 0 0;
                0 -1 0;
                0 0 1];
            
%J = zeros(6,6+n);
J = zeros(6,6);


%Jman = J_man(DH);
R_B_I = Rpy2Rot(eta2);
R_0_I = R_B_I*R_0_B;


r_B0_I = R_B_I*r_B0_B;
eta_0ee_I = eta_ee1 - eta1 - r_B0_I;

% Position Jacobian
J(1:3,1:3)   = R_B_I;
J(1:3,4:6)   = -(S(r_B0_I)+S(eta_0ee_I))*R_B_I;

% Orientation Jacobian
J(4:6,1:3)   = zeros(3,3);
J(4:6,4:6)   = R_B_I;

format long
J