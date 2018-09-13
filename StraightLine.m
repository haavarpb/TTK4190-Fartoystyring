%% Data init
N = 200;
h = 0.1;
U = 5;
delta = 0;

%Nomoto 
T = 20;
K = 0.1;
bias = 0.001;

numerator1 = [K*U];
denominator1 = [T 1 0 0];
h1 = tf(numerator1,denominator1);

numerator2 = [U];
denominator2 = [T 1 0 0];
h2 = tf(numerator2,denominator2);

%Initial conditions
x0 = 0; % Meters
y0 = 100; % Meters
psi0 = 0*deg2rad; % rad
r0 = 0*deg2rad; % rad/s

%Velocity equations
x_dot = @(psi, U) [U*cos(psi)];
y_dot = @(psi, U) [U*sin(psi)];

%Storage
table = zeros(N+1, 2);        % memory allocation

%Cross track error
% y = K*U*(T*exp(-t/T)+t-T)*delta+U*(T*exp(-t/T)+t-T)*bias;

for i = 1:N+1
    
    t = (i-1)*h; 
    
    %Simulation
    
    
    table(i, :) = [x, y]
    
    %Integration
    y_int = trapz(table(:, 2)); % Integrerer opp lagret y
    
    %PID
    delta = -k_p*y-y_dot
end
