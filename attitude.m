%%%% Possible something wrong with the desired angles, looks better with
%%%% deg2rad. 

close all;

% M-script for numerical integration of the attitude dynamics of a rigid 
% body represented by unit quaternions. The MSS m-files must be on your
% Matlab path in order to run the script.
%
% System:                      .
%                              q = T(q)w
%                              .
%                            I w - S(Iw)w = tau
% Control law:
%                            tau = constant
% 
% Definitions:             
%                            I = inertia matrix (3x3)
%                            S(w) = skew-symmetric matrix (3x3)
%                            T(q) = transformation matrix (4x3)
%                            tau = control input (3x1)
%                            w = angular velocity vector (3x1)
%                            q = unit quaternion vector (4x1)
%
% Author:                   2018-08-15 Thor I. Fossen and H�kon H. Helgesen

%% USER INPUTS
h = 0.1;                     % sample time (s)
N  = 12000;                    % number of samples. Should be adjusted

% model parameters
I = diag([50 100 80]);       % inertia matrix
I_inv = inv(I);

% constants
deg2rad = pi/180;   
rad2deg = 180/pi;

phi = -5*deg2rad;            % initial Euler angles
theta = 10*deg2rad;
psi = -20*deg2rad;




q = euler2q(phi,theta,psi);   % transform initial Euler angles to q

%For Non-zero constant reference signal
% phi_ref = 10*deg2rad;
% phi_error = phi_ref-phi;
% q = euler2q(phi_error,theta,psi);   % transform initial Euler angles to q

w = [0 0 0]';                 % initial angular rates

table = zeros(N+1,14);        % memory allocation

% Control parameters
K_d = eye(3)*40;
k_p = 2;

% Tracking param
N_array = 1:(N + 1);
phi_d = 0*N_array;
theta_d = deg2rad*15*cosd(0.1*N_array);
psi_d = deg2rad*10*sind(0.05*N_array);

phi_dot_d = 0*N_array;
theta_dot_d = deg2rad*-1.5*sind(0.1*N_array);
psi_dot_d = deg2rad*(1/2)*cosd(0.05*N_array);

T_f2b = @(phi, theta, psi) [1 0 -sin(theta); 0 cos(phi) cos(theta)*sin(phi); 0 -sin(phi) cos(theta)*cos(phi)];

%% FOR-END LOOP
for i = 1:N+1
%     q_d = euler2q(0, theta_d(i), psi_d(i)); 
%     q_tilde = quatmultiply(quatconj(q_d'), q')';
%     e_tilde = q_tilde(2:end);
%    
%     w_d = T_f2b(phi_d(i), theta_d(i), psi_d(i))*[phi_dot_d(i) theta_dot_d(i) psi_dot_d(i)]';
%     w_tilde = w - w_d;
    t = (i-1)*h;                  % time
    %tau = - K_d*w_tilde - k_p*e_tilde;       % control law
    tau = - K_d*w - k_p*q(2:end,1); 
    [phi,theta,psi] = q2euler(q); % transform q to Euler angles
    [J,J1,J2] = quatern(q);       % kinematic transformation matrices

    q_dot = J2*w;                        % quaternion kinematics
    w_dot = I_inv*(Smtrx(I*w)*w + tau);  % rigid-body kinetics

    table(i,:) = [t q' phi theta psi w' tau'];  % store data in table

    q = q + h*q_dot;	             % Euler integration
    w = w + h*w_dot;

    q  = q/norm(q);               % unit quaternion normalization
end 

%% PLOT FIGURES
t       = table(:,1);  
q       = table(:,2:5); 
phi     = rad2deg*table(:,6);
theta   = rad2deg*table(:,7);
psi     = rad2deg*table(:,8);
w       = rad2deg*table(:,9:11);  
tau     = table(:,12:14);
% w_tilde     = table(:, 15:17);

%Calcuklating error
phi_err = phi; %Since phi_d = 0
theta_err = theta - rad2deg*theta_d' ;
psi_err = psi - rad2deg*psi_d';

figure (1); clf;
hold on;
plot(t, phi, 'b');
plot(t, theta, 'r');
plot(t, psi, 'g');
hold off;
grid on;
legend('\phi', '\theta', '\psi');
title('Euler angles');
xlabel('time [s]'); 
ylabel('angle [deg]');

% figure (999); clf;
% hold on;
% plot(t, phi_err', 'b');
% plot(t, theta_err', 'r');
% plot(t, psi_err', 'g');
% hold off;
% grid on;
% legend('\phi', '\theta', '\psi');
% title('Error in euler angles');
% xlabel('time [s]'); 
% ylabel('angle [deg]');

figure (2); clf;
hold on;
plot(t, w(:,1), 'b');
plot(t, w(:,2), 'r');
plot(t, w(:,3), 'g');
hold off;
grid on;
legend('x', 'y', 'z');
title('Angular velocities');
xlabel('time [s]'); 
ylabel('angular rate [deg/s]');

% figure (161); clf;
% hold on;
% plot(t, w_tilde(:,1), 'b');
% plot(t, w_tilde(:,2), 'r');
% plot(t, w_tilde(:,3), 'g');
% hold off;
% grid on;
% legend('x', 'y', 'z');
% title('Angular velocities error');
% xlabel('time [s]'); 
% ylabel('angular rate [deg/s]');

figure (3); clf;
hold on;
plot(t, tau(:,1), 'b');
plot(t, tau(:,2), 'r');
plot(t, tau(:,3), 'g');
hold off;
grid on;
legend('x', 'y', 'z');
title('Control input');
xlabel('time [s]'); 
ylabel('input [Nm]');

% figure (4); clf;
% hold on;
% plot(t, rad2deg*theta_d', 'r');
% plot(t, rad2deg*psi_d', 'g');
% plot(t, rad2deg*phi_d', 'b');
% hold off;
% grid on;
% legend('\theta', '\psi', '\phi');
% title('Desired trajectory in euler angles');
% xlabel('time [s]'); 
% ylabel('angle [deg]');