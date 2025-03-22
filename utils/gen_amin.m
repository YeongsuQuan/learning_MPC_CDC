function [a_min_update] = gen_amin(t_acc,ts,M,xt,a_min,p_obs)


% System model - continuous time
Act     = [0 1 0;
           0 0 1;
           0 0 -t_acc];
Bct     = [0;0;t_acc];

Cct     = [1 0 0];

Dct     = zeros(1,1);

% Discrete-time model with zoh
Model           =   c2d(ss(Act,Bct,Cct,Dct),ts);  

% System model matrices (discrete time):
[A,B,C,D]       =   ssdata(Model);                  % Model matrices 

% Signal dimensions
nx      =   size(A,1);      % number of states
nu      =   size(B,2);      % number of inputs

% Constraints 
H_1      =   -1;
H_2      =   [0 -1 0];
H_3      =   [0 0 -1];
H_4      =   [0 1 0];
G        =   [1 0 0];
b        =   p_obs;

               
nqG    =   size(G,1);
nqH1   =   size(H_1,1);
nqH2   =   size(H_2,1);
nqH3   =   size(H_3,1);

[~,~,Lambda_x,Gamma_x]     =   Traj_matrices(M,A,B,C,D);
Gamma_x_M     =   Gamma_x(nx*M+1:nx*(M+1),:);
Lambda_x_M    =   Lambda_x(nx*M+1:nx*(M+1),:);
G_bar                                   =   zeros((M+1)*nqG,(M+1)*nx);
b_bar                                   =   zeros((M+1)*nqG,1);
H2_bar                                  =   zeros((M+1)*nqH2,(M+1)*nx);
H3_bar                                  =   zeros((M+1)*nqH3,(M+1)*nx);
H1_bar                                  =   zeros(M*nqH1,M*nu);
Ha_bar_u                                =   ones(M,1);
Ha_bar_x                                =   ones(M+1,1);

for ind = 1:M+1
    G_bar((ind-1)*nqG+1:ind*nqG,(ind-1)*nx+1:ind*nx)        =   G;
    b_bar((ind-1)*nqG+1:ind*nqG,1)                          =   b;
    H2_bar((ind-1)*nqH2+1:ind*nqH2,(ind-1)*nx+1:ind*nx)       =   H_2;
    H3_bar((ind-1)*nqH3+1:ind*nqH3,(ind-1)*nx+1:ind*nx)       =   H_3;
end
for ind = 1:M
    H1_bar((ind-1)*nqH1+1:ind*nqH1,(ind-1)*nu+1:ind*nu)        =   H_1;
end

C_1   =   [G_bar*Gamma_x;
           H2_bar*Gamma_x];
C_2   =   H_4*Gamma_x_M;
C_3u  =   [H1_bar;
           H3_bar*Gamma_x];
C_3a  =   [Ha_bar_u;
           Ha_bar_x];

d_1   =   [b_bar-G_bar*Lambda_x*xt;
           -H2_bar*Lambda_x*xt];
d_2   =   -H_4*Lambda_x_M*xt;
d_3   =   [zeros(size(Ha_bar_u,1),1);
           -H3_bar*Lambda_x*xt];

Aneq   =   [C_1    zeros(size(C_1,1),size(C_3a,2));
            C_3u   C_3a];
bneq   =   [d_1;
            d_3];

Aeq    =   [C_2 0];
beq    =   [d_2];

% Cost
H_qp   = zeros(M+1,M+1);
H_qp(M+1,M+1) = 2;
f_qp   = [zeros(1,M) -2*a_min];

% QP
options = optimoptions('quadprog','Display','none');

U = quadprog(H_qp,f_qp,Aneq,bneq,Aeq,beq,[],[],[],options);

% Opt val
a_min_update = [zeros(1,M) 1]*U;

end

