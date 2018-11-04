%% The function generate the state transition matrices for LQR
%  X_dot = A*X+B*U
%  Y= C*X

function [A,B,C]= state_matrices(mass,g,I,psi,d_psi)

    A11 = zeros(6,6);
    A12 = eye(6,6);

    C1 = (I(2,2)-I(3,3))/I(1,1);
    C2 = (I(1,1)-I(3,3))/I(2,2);
    A21 = [ 0 0 0 g*sin(psi)    g*cos(psi)  0;
            0 0 0 -g*cos(psi)   g*sin(psi)  0;
            0 0 0 0             0           0;
            0 0 0 C1*d_psi^2    0           0;
            0 0 0 0             C2*d_psi^2  0;
            0 0 0 0             0           0];

    A22 = [ 0 0 0 0             0           0;
            0 0 0 0             0           0;
            0 0 0 0             0           0;
            0 0 0 0             C1*d_psi    0;
            0 0 0 -C2*d_psi     0           0;
            0 0 0 0             0           0];

    A = [A11 A12;
         A21 A22];

    % matrix B
    B1 = zeros(8,4);
    B2 = [1/mass 0         0               0;
          0     1/I(1,1)  0               0;
          0     0         1/I(2,2)        0;
          0     0         0         1/I(3,3)   ];

    B = [B1;B2];

    C11 = [ 1 0 0 0 0 0;
            0 1 0 0 0 0;
            0 0 1 0 0 0;
            0 0 0 0 0 1];
    C12 = zeros(4,6);
    C = [C11 C12];
end