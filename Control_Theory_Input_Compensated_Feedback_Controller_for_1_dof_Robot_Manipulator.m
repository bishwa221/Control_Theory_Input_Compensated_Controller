clear all
% please use appropriate check points at the end of each section to see the
% output of each section clearly

%% openloop tranfer function
nume1 = [0.023];
deno1 = [0.02913 0.1543 0.1205 0];
sysm1 = tf(nume1,deno1)

%% openloop response
step(sysm1)
stepinfo(sysm1)

%% closed loop transfer function
nume = [0.023];
deno = [0.02913 0.1543 0.1205 0.001534];
sysm = tf(nume,deno)
[A,B,C,D] = tf2ss(nume,deno)
A
B
C
D

%% check controllability and observability of closed loop system
Qc =  [B A*B A*A*B ]
rankQc = rank(Qc)

Qo = [C; C*A; C*A*A]
rankQo = rank(Qo)

%% eigen values of A
[V,value] = eig(A)
% stable system

%% closed loop response
step(sysm)
%hold off
stepinfo(sysm)

%% placement of poles using ackerman formula
poles_L = [-2.9-1.3i,-2.9+1.3i,-6.2];
L = acker(A',C',poles_L); % L is the Kc matrix
L
A1 = A - B*L;
sys = ss(A1, B, C, D);
step(sys)
stepinfo(sys)
% Tracking problem is visible. output cannot track unit step input exactly

%% State feedback controller with Input compensation
T = C * inv([ A- B*L]) * B;
Ts = -1/T
a_new = A - B*L
b_new = B*Ts
sys_final = ss(a_new, b_new, C, D);
step(sys_final)
stepinfo(sys_final)

%% Lyapunov stability check
a_new= A- B*L;
MatrixQ = [1 0 0;0 1 0; 0 0 1];
MatrixA = a_new;
MatrixX = lyap(MatrixA,MatrixQ);
disp('Lyapunov Solution is MatrixX = ');
disp(MatrixX);
k = eig(MatrixX);
disp('Eigen values of Lyapunov: ');
disp(k);
if(k(1)>0 && k(2)>0)
disp('The System is Positive Definite and hence stable');
else
disp('The System is not Positive Definite and hence unstable');
end

%% compare with PID
[b,a] = ss2tf(a_new,b_new,C,D);
G = tf(b,a);
[co,info] = pidtune(G,'PID')
Tref = getPIDLoopResponse(co,G,"closed-loop");
Tdist = getPIDLoopResponse(co,G,"input-disturbance");
step(Tref,Tdist,sys_final)
stepinfo(Tref)
legend("PID Reference Tracking","Disturbance Rejection", "Proposed Controller")

%% State transition
X = [];
X_0 = [1;1;1];
for t = 0:0.1:5
    EA = expm(a_new * t);
    X = [X, EA*X_0]; %(3,100)
end
%% Plot the states
t = [0:0.1:5]; 
X1 = [1 0 0 ]*X;
X2 = [0 1 0]*X;
X3 = [0 0 1]*X;
figure
plot(t,X1,'r')
hold on
plot(t,X2,'k')
hold on
plot(t,X3,'m')
hold off
title('X vs t')
xlabel('Time(s)')
ylabel('X')
legend('X1','X2','X3')

