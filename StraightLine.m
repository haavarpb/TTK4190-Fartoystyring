%% Data init
N = 10000;
h = 0.1;
U = 5;
delta = 0;

% Constants
deg2rad = pi/180;   
rad2deg = 180/pi;

%Nomoto 
T = 20;
k = 0.1;
bias = 0.001;

numerator1 = [k*U];
denominator1 = [T 1 0 0];
h1 = tf(numerator1,denominator1);

numerator2 = [U];
denominator2 = [T 1 0 0];
h2 = tf(numerator2,denominator2);

%Initial conditions
x0 = 0; % Meters
y0 = 100; % Meters
psi0 = 0; % rad
r0 = 0; % rad/s

%Velocity equations
x_dot = @(psi, U) [U*cos(psi)];
y_dot = @(psi, U) [U*sin(psi)];

%Storage
x_dot_store = zeros(N+1, 1);        % memory allocation
y_dot_store = zeros(N+1, 1);        % memory allocation
x_store = zeros(N+1, 1);        % memory allocation
y_store = zeros(N+1, 1);        % memory allocation
table = zeros(N+1, 4);        % memory allocation

%PID init
k_p = 0.001;
k_i = 0.0000003;
k_d = 0.05;

%Cross track error
% y = K*U*(T*exp(-t/T)+t-T)*delta+U*(T*exp(-t/T)+t-T)*bias;

% State init
x_store(1) = x0;
y_store(1) = y0;
x_dot_store(1) = 0;
y_dot_store(1) = 0;
psi = 0;
r_dot = 0;
r = 0;

for i = 1:N+1
    
    t = (i-1)*h; 
    
    %PID
    delta = -k_p*y_store(i)-k_d*U*psi-k_i*trapz(y_store);
    
    delta = min([abs(delta) 20])*sign(delta);
    
    %Simulation
    r_dot = k*delta/T + bias/T - r/T;
    r = euler2(r_dot, r, h);
    
    
%     r = exp(-t/T)*(bias+delta*k)/T;

    psi = euler2(r, psi, h); % integrere 
    
%     psi = k*(1-exp(-t/T))*delta+bias*(1-exp(-t/T));
    
    
    x_dot_store(i+1) = x_dot(psi, U);
    y_dot_store(i+1) = y_dot(psi, U);
    
    %Obtainging position
    x_store(i+1) = euler2(x_dot_store(i+1),x_store(i),h); % Integrerer opp x_dot
    y_store(i+1) = euler2(y_dot_store(i+1),y_store(i),h); % Integrerer opp y_dot
    
    table(i, :) = [t, psi, delta, r];
    
end

%% PLOT FIGURES
t = table(:,1);  
psi = table(:, 2);
delta = table(:, 3);
r = table(:, 4);

x_store = x_store(1:end-1);
y_store = y_store(1:end-1);
x_dot_store = x_dot_store(1:end-1);
y_dot_store = y_dot_store(1:end-1);

figure (1); clf;
plot(t, psi, 'b');
grid on;
legend('\psi');
title('Yaw angle');
xlabel('time [s]'); 
ylabel('Degrees');

figure (2); clf;
hold on;
plot(t, delta, 'b');
hold off;
grid on;
legend('\delta');
title('Input');
xlabel('time [s]'); 
ylabel('Degrees');

figure (3); clf;
hold on;
plot(t, r, 'b');
hold off;
grid on;
legend('$\dot{\psi}$','Interpreter','latex');
title('Yaw rate');
xlabel('time [s]'); 
ylabel('Degrees/s');

figure (100); clf;
hold on;
plot(x_store, y_store, 'b');
hold off;
grid on;
legend('x');
title('Position plot');
xlabel('x [m]'); 
ylabel('y [m]');

