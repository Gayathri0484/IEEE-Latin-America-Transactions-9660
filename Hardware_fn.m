function [th,u]= fcn(X,ur,k,c)
% Motor
% Resistance
Rm = 8.4;
kt = 0.042;
% Back-emf constant 
km = 0.042;
% Rotary Arm
% Mass 
mr = 0.095;
% Total length 
rr = 0.085;
% Moment of inertia about pivot
Jr = mr*rr^2/3;
% Equivalent Viscous Damping Coefficient
br = 1e-3; 
% Pendulum Link
% Mass
mp = 0.024;
% Total length
Lp = 0.129;
% Pendulum center of mass 
l = Lp/2;
% Moment of inertia about pivot
Jp = mp*Lp^2/3;
% Equivalent Viscous Damping Coefficient 
bp = 5e-5; 
% Gravity Constant
g = 9.81;
Jt = Jr*Jp - mp^2*l^2*rr^2;

del_t =0.01; %sampling time 


%Discrete state space matrices
Ap=[1 0.0073 0.0094 0; 0 1.012 -0.0006 0.01; 0 1.430 0.884 0.0026; 0 2.55 -0.114 1.008];
Bp=[0.0024; 0.0024;0.4766;0.4720];
Cp=[1 0 0 0];
Dp=[0];
[m1,n1]=size(Cp); 
[n1,n_in]=size(Bp);

%Augmented state space matrices
A_e=[1 0.0073 0.0094 0 0; 0 1.012 -0.0006 0.01 0; 0 1.43 0.884 0.0026 0; 0 2.559 -0.114 1.0081 0; 1 0.0073 0.0094 0 1];
B_e=[0.0024; 0.0024; 0.4766; 0.4720;0.0024];
C_e=[0 0 0 01];

N_sim=100; % simulation instances
Np=50; % prediction horizon
Nc=10;% control horizon
n=n1+m1;
h=zeros(50,5);
h(1,:)=[0 0 0 0 1]; 
F=zeros(50,5);
F(1,:)=[1 0.0073 0.0094 0 1];
for kk=2:Np
h(kk,:)=h(kk-1,:)*A_e;
F(kk,:)= F(kk-1,:)*A_e;
end
v=h*B_e;
Phi=zeros(Np,Nc); %declare the dimension of Phi
Phi(:,1)=v; % first column of Phi
for i=2:Nc
Phi(:,i)=[zeros(i-1,1);v(1:Np-i+1,1)]; %Toeplitz matrix
end
BarRs=ones(Np,1); 
Phi_Phi= Phi'*Phi;
Phi_F= Phi'*F;
Phi_R=Phi'*BarRs;


[n,n_in]=size(B_e);
xm_old=X %sensor data stored as old data
u=ur

Rbar=0.2 %weight for del U

%Constraints
u_max=10;
u_min=-10;


%% Linearised model + coupling nonlinearities is needed
d1=-(0.5*mp*Lp*Lp*X(2)*X(3)*X(4))-(0.5*mp*rr*Lp*X(2)*X(4)*X(4))
d2=(0.25*mp*Lp*Lp*X(2)*X(3)*X(3))
d=[0;0;d1;d2] % coupling nonlinerity 
xm=Ap*X+Bp*u+(d)


y=Cp*xm %new data
Xf=[xm-X;y]
r=k;
th=Xf(3);

% Warm start
lqrmat=[0.5;0.22;0.04;-0.037;-0.079];
if(c<5)
for c= 1:5
 deltau= lqrmat(c);
end
end
if(c>=5)
DeltaU=inv(Phi_Phi+Rbar*eye(Nc,Nc))*(Phi_R*r(c)-Phi_F*Xf)
deltau=DeltaU(1,1)
end

u=u+deltau
if (u>u_max) 
    deltau=u_max-u;
u=u_max;
end
if (u<u_min) 
   deltau=u_min-u;
u=u_min; end