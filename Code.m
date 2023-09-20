A = [0 1 0 0;
-40 -1 40 0;
0 0 0 1;
10 0 -10 0];
B = [0;0;0;0.5];
C = [1 0 0 0];
D = 0;
sys = ss(A,B,C,D);

%observatory
phi_o = obsv(A,C);
disp('phi_o');
disp(phi_o);
disp('rank phi_o');
disp(rank(phi_o));

%controllability
phi_c = ctrb(A,B);
disp('phi_c');
disp(phi_c);
disp('rank phi_c');
disp(rank(phi_c));

% Jordan Canonical Form
Jsys = canon(sys,"Modal");

%Eigenvalues
eigenvalues = eig(A)

% Controllability Canonical Realization
Csys = canon(sys,"Companion");

%Transfer Function
transferfunc = tf(sys);

%Observability Realization
Osys = transpose (Csys);

%Step Response to Open Loop system
figure(1);
step(transferfunc);
stepinfo(sys)

%Step Response to Close Loop system(state feedback)
K_1 = place(A,B,[-1+2j -1-2j -1 -2]); %near jw
A_cl_1 = A-B*K_1;
sys_c_1 = ss(A_cl_1,B,C,D);
figure(2)
step(sys_c_1)
stepinfo(sys_c_1)

%Step Response to Close Loop system(state feedback)
K_2 = place(A,B,10000*[-1+2j -1-2j -1 -2]); %far from jw
A_cl_2 = A-B*K_2;
sys_c_2 = ss(A_cl_2,B,C,D);
figure(3)
step(sys_c_2)
stepinfo(sys_c_2)

%Observer
P_O = 1000*[-1+2i, -1-2i, -2, -4];
L = acker(A', C', P_O)';
A_OC = [A -B*K_1; L*C A-L*C-B*K_1];
B_OC = [B; zeros(4,1)];
C_OC = eye(8);


X0C = [1, -2, 2, -3, 0, 0, 0, 0];
sys_OC = ss(A_OC, B_OC, C_OC, 0);
t_OC = 0:0.00000001:0.001;
u_OC = 0 * ones(1,length(t_OC));

[Y,T,X] = lsim(sys_OC, u_OC, t_OC, X0C);



figure(4)
subplot(2,2,1)
plot(t_OC, X(:,1),'b')
hold on
plot(t_OC, X(:,5),'k--')

subplot(2,2,2)
plot(t_OC, X(:,2),'b')
hold on
plot(t_OC, X(:,6),'k--')

subplot(2,2,3)
plot(t_OC, X(:,3),'b')
hold on
plot(t_OC, X(:,7),'k--')

subplot(2,2,4)
plot(t_OC, X(:,4),'b')
hold on
plot(t_OC, X(:,8),'k--')
