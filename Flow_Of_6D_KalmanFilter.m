clear all;
clc;

measurements = csvread('C:\My_Folder\Apollo_Docs\measurement.csv');

x = measurements(:,1);
y = measurements(:,2);

delT = 5;

sigma = 50;
theta = 60;
vel = 314;
acc = 9.12;

F = [1 0 delT 0 (0.5*delT^2) 0; 0 1 0 delT 0 (0.5*delT^2); 0 0 1 0 delT 0; 0 0 0 1 0 delT; 0 0 0 0 exp(-delT/theta) 0; 0 0 0 0 0 exp(-delT/theta)]; % state transition matrix
P = [sigma^2 0 0 0 0 0; 0 sigma^2 0 0 0 0; 0 0 vel^2 0 0 0; 0 0 0 vel^2 0 0; 0 0 0 0 acc^2 0; 0 0 0 0 0 acc^2]; % Initial Process covariance matrix
X = [3353.1273; 2395.5509; 343.6778; 488.7182; 0.9032; -7.3500]; % initial state vector
Q = [0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 (acc^2*(1-exp(-2*delT/theta))) 0; 0 0 0 0 0 (acc^2*(1-exp(-2*delT/theta)))]; % Process noise covariance matrix
w = [0; 0; 0; 0; 6.55518382; 4.44595807];

H = [1 0 0 0 0 0; 0 1 0 0 0 0]; % Measurement matrix
R = [sigma^2 0; 0 sigma^2]; % Measurement noise covariance matrix
v = [6.72063866; 15.66966363];

I = eye(length(X));

plot_predicted_state = false;
plot_measured_state = true;
plot_corrected_state = true;

for i=2:length(x)
    % Prediction    
    % Prediction of state vector
    %X = F*X + w;
    X = F*X;
    % Prediction of process covaraiance matrix
    P = F*P*F' + Q;
    
    % Plot predicted state
    if plot_predicted_state
        scatter(X(1), X(2), 'r');
        hold on;
    end

    % Calculate Kalman Gain
    S = (H*P*H') + R;
    K = (P*H')/S;

    % Update
    z = [x(i); y(i)];
    %Y = z - ((H*X) + v);
    Y = z - (H*X);
    
    % Current state, correction step
    % Corrct the state vector
    X = X + K*Y;
    % Correct the process covariance matrix
    P = (I - (K*H))*P;    
    
    % Plot measured state    
    if plot_measured_state
        scatter(x(i), y(i), 'g');
        hold on;
    end
    % Plot corrected state
    if plot_corrected_state
        scatter(X(1), X(2), 'b');
        hold on;
    end
end