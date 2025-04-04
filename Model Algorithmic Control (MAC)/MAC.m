clc ; 
clear;
close all;
%%
T0 = 0 ;
Ts = 0.1 ;
Tf = 100 ;
t = T0:Ts:Tf ; t = t' ;
system = tf(1,[1 3 2]) ; % system = (1)/(s^2+3s+2) ;
% impulse (system) ;
Info = stepinfo(system) ;
N = floor(Info.SettlingTime/Ts) + 5 ;
p = 10 ;
m = 5 ;
L = 1 ; % Landa
h = impulse(system , t) ; h = h(2:end) ;
system_dis = c2d(system , Ts) ; % y(k) = 1.724y(k-1) - 0.7408y(k-2) + 0.004528u(k-1) + 0.004097u(k-2) ;
% A = [ 1 -1.724 0.7408] ;
% B = [ 0.004528 0.004097] ;
A = system_dis.den ; A = cell2mat(A) ;
B = system_dis.num ; B = cell2mat(B) ; B = B(2:end) ;
H1 = zeros( p , m) ;
H2 = zeros (p , N-1) ;
for i = 1:m
    H1(: , 1) = [zeros(i-1 , 1) ; h(1:p-i+1)] ;
end
for i = 1:m
    H2(i , :) = [zeros(i-1 , 1) ; h(N:-1:i+1)] ;
end
%% Main Loop
Nt = numel (t) ;
W = [ones(floor(Nt/4) , 1) ; 2*ones(floor(Nt/4) , 1) ; -1*ones(floor(Nt/4) , 1) ; zeros(floor(Nt/4) , 1)] ;
y = zeros(Nt , 1) ;
u = zeros(Nt , 1) ;
DegSys = numel(A) - 1 ;
Uminus = zeros(N-1 , 1) ;
for i = 3:Nt-1
    y(i) = 1.724*y(i-1) - 0.7408*y(i-2) + 0.004528*u(i-1) + 0.004097*u(i-2) ;
    % y(i) = -A(2:end)*y(i-1:-1:i-DegSys)' + B * u(i-1:-1:i-DegSys) ;
    for j = 1:N-1
        if i-N+j < 1
            Uminus(j , 1) = 0 ;
        else
            Uminus(j , 1) = u(i-N+j) ;
        end
    end
    FreeResponse = H2 * Uminus ;
    dUopt = (H1'*H1 + L*eye(m,m))\(H1' * (W(i) - FreeResponse)) ;
    u(i) = dUopt(1) ;
end
%% plot results
figure(1) ;
subplot(2,1,1) ;
plot(t(1:Nt-1) , W(1:Nt-1) , t(1:Nt-1) , y(1:Nt-1) * W(20/Ts)/y(20/Ts) , 'LineWidth' , 2);
xlabel('Time (second)') ;
ylabel('Amp y') ;
legend('Ref' , 'Out') ;
title('Output Signal (Mac)') ;
grid on

subplot(2,1,2) ;
plot(t(1:Nt-1) , u(1:Nt-1) *W(20/Ts)/y(20/Ts) , 'Linewidth' , 2) ;
xlabel('Time (second)') ;
ylabel('Amp u') ;
title('Effort control Response (Mac)') ;
legend('U') ;
grid on

