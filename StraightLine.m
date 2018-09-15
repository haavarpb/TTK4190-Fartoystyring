%% Data init
N = 200;
h = 0.1;
U = 5;
delta = 0;

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
table = zeros(N+1, 3);        % memory allocation

%PID init
k_p = 0.001;
k_i = 1;
k_d = 1;

%Cross track error
% y = K*U*(T*exp(-t/T)+t-T)*delta+U*(T*exp(-t/T)+t-T)*bias;

% State init
x_store(1) = x0;
y_store(1) = y0;
x_dot_store(1) = 0;
y_dot_store(1) = 0;
psi = 0;

for i = 1:N+1
    
    t = (i-1)*h; 
    
    %PID
    delta = -k_p*y_store(i);%-y_dot_store(i);
    
    %Simulation
    psi = k*(1-exp(-t/T))*delta+bias*(1-exp(-t/T));
    
    x_dot_store(i+1) = x_dot(psi, U);
    y_dot_store(i+1) = y_dot(psi, U);
    
    %Obtainging position
    x_store(i+1) = trapz(h, x_dot_store); % Integrerer opp x_dot
    y_store(i+1) = trapz(h, y_dot_store); % Integrerer opp y_dot
    
    table(i, :) = [t, psi, delta];
    
end

%% PLOT FIGURES
t = table(:,1);  
psi = table(:, 2);
delta = table(:, 3);

x_store = x_store(1:end-1);
y_store = y_store(1:end-1);
x_dot_store = x_dot_store(1:end-1);
y_dot_store = y_dot_store(1:end-1);

figure (1); clf;
hold on;
plot(t, x_store, 'b');
plot(t, y_store, 'r');
plot(t, x_dot_store, 'g');
hold off;
grid on;
legend('x', 'y', 'x_dot');
title('Test');
xlabel('time [s]'); 
ylabel('m');

figure (2); clf;
hold on;
plot(y_store, x_store, 'b');
hold off;
grid on;
legend('x');
title('Position plot');
xlabel('x [m]'); 
ylabel('y [m]');

