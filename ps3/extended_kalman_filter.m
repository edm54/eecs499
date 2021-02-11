%{
Eric Miller
Edm54
EECS 499
Project 1
Extended Kalman Filter
%}

set(0, 'DefaultAxesFontWeight', 'normal', ...
      'DefaultAxesFontSize', 18, ...
      'DefaultAxesFontAngle', 'normal', ... 
      'DefaultAxesFontWeight', 'normal', ... 
      'DefaultAxesTitleFontWeight', 'bold', ...
      'DefaultAxesTitleFontSizeMultiplier', 1.2) ;
set(groot,'defaultLineLineWidth',3)

%% initialize parameters
alpha = [.0001, .0001, .01, .0001, .0001, .0001];
map = [[0,0]; [4, 0]; [8,0]; [8,6]; [4,6];[0,6]];
u = [[1,0]; [1,0]; [1,0]; [pi/2, pi/2]; [pi/2, pi/2]; [1,0]; [1,0]; [1,0]];
z = [[2.276, 5.249, 2]; [4.321, 5.834, 3]; [3.418, 5.87, 3]; [3.774, 5.91, 4]; [2.631, 5.14, 5]; [4.77, 5.791, 6];[3.828, 5.74, 6]; [3.153, 5.739, 6]]
sigma_r = .1
sigma_phi = .09
variance_r = sigma_r^2
variance_phi = sigma_phi^2
big_q = eye(2) .* [variance_r ;variance_phi]

%% Plot landmarks and noiseless trajectory
x0 = [2, 2, 0];
x = [x0];
state = x0;
t = 1;
figure
hold on
plot(state(1), state(2), 'r*');
for i = 1:8
    [x1, y1, theta_1] = find_x_prime(state, u(i, :), t);
    state = [x1, y1, theta_1];
    x = [x ;state];
    plot(state(1), state(2), 'r*');
end

for i = 1:6
    local_m = map(i, :);
    plot(local_m(1), local_m(2), 'k*', 'LineWidth', 10)  ;  
end

%% Kalman Filter Algo

% Since no motion, we assume the covariance matrix is the identity matrix
% (which assumes initially independent measurements)
alphas = [alpha(1)+alpha(2);  ( alpha(3) + alpha(4)); alpha(5) + alpha(6)];
cov_t = eye(3,3) .* alphas; 
low_ind = 1;
state = x(low_ind, :);
s_arr = [state];
noisy_state = [state];
zs = []

for state_index = low_ind:8
    %% Prediction
    g = calculate_G(u(state_index,:), state, 1);
    m = calculate_M(u(state_index,:),alpha);
    v = calculate_V(u(state_index,:), state, 1);

    cov_new_est = g * cov_t * g' + v * m * v';

    [xp, yp, tp] = find_x_prime(state, u(state_index,:), t)
    mu_new_est = [xp, yp, tp];
    noisy_state = [noisy_state; mu_new_est];
    plot_error_elipse(mu_new_est, cov_new_est, 'r')

    %% Correction
    current_z = z(state_index, 1:2);
    current_map = map(z(state_index,3), :);
    [true_range, true_bearing] = get_true_range_bearing(current_map, mu_new_est);
    z_hat = [true_range, true_bearing];
    zs = [zs ;z_hat]
    q = true_range^2;
    h = calculate_H(current_map, mu_new_est, q);
    s = h * cov_new_est * h' + big_q;

    k = cov_new_est  * h' * inv(s);
    state = mu_new_est + (k * (current_z' - z_hat'))';

    cov_t = (eye(3) - k * h) * cov_new_est;
    s_arr = [s_arr ;state];
    plot_error_elipse(state, cov_t, 'b')
end

%% Plot results
x0 = [2, 2, 0];
x = [x0];
state = x0;
t = 1;
figure
hold on
plot(state(1), state(2), 'r*');
for i = 1:8
    [x1, y1, theta_1] = find_x_prime(state, u(i, :), t);
    state = [x1, y1, theta_1];
    x = [x ;state];
    plot(state(1), state(2), 'r*');
end

for i = 1:6
    local_m = map(i, :);
    plot(local_m(1), local_m(2), 'k*', 'LineWidth', 10)  ;  
end

hold on
for s = 1:10-low_ind
    plot(s_arr(s, 1), s_arr(s, 2), 'b*')
    plot(noisy_state(s, 1), noisy_state(s, 2), 'g*')
    
end



%% Functions
function [true_range, true_bearing] = get_true_range_bearing(m, state)
   landmark_y = m(2);
   landmark_x = m(1);
   
   robot_x = state(1);
   robot_y  = state(2);
   robot_theta = state(3);
   
   true_range = sqrt((landmark_x - robot_x)^2 + (landmark_y - robot_y)^2);
   true_bearing = atan2(landmark_y - robot_y, landmark_x - robot_x) - robot_theta;
   
   if true_bearing<0
       true_bearing = 2 * pi + true_bearing;
   end
end

function h = calculate_H(map, mu, q)
    h = ones(2, 3);
%     H(3, 1:3) = 0;
    h(1, 3) = 0;
    
    map_x = map(1);
    map_y = map(2);
    
    mu_x = mu(2);
    mu_y = mu(2);
    
    h(1,1) = -1 * (map_x - mu_x)/sqrt(q);
    h(1, 2) = -1 * (map_y - mu_y)/sqrt(q);
    
    h(2, 1) = (map_y - mu_y)/q;
    h(2, 2) = -1 * (map_x - mu_x)/q;
    h(2, 3) = -1;
   
end


function g = calculate_G(u, mu, t)
    v = u(1);
    w = u(2);
    
    mu_theta = mu(3);
    
    g = ones(3, 3);
    g(2, 1) = 0;
    g(3, 1) = 0;
    g(3, 2) = 0;
    g(1, 2) = 0;
    if w == 0
        r = 1;
        g(1, 3) =  0; 
        g(2, 3) =  0;
    else
        r = v/w;
        
        dx_d_mu = r * (-cos(mu_theta) + cos(mu_theta + w * t));
        dy_d_mu = r * (-sin(mu_theta) + sin(mu_theta + w * t));
    
        g(1, 3) =  dx_d_mu; 
        g(2, 3) =  dy_d_mu;
    end
    
    
end

function v_mat = calculate_V(u, mu, t)
    v = u(1);
    w = u(2);
    
    theta = mu(3);
    
    if w == 0
        v_mat = zeros(3, 2);
        v(3,2) = t;
    else
        v_mat = ones(3, 2);
        v_mat(3,1) = 0;
        v_mat(3,2) = t;
        v_mat(1,1) = -1 * (sin(theta) + sin(theta + w * t))/w;
        v_mat(1,2) = (v * (sin(theta) - sin(theta + w * t))/w^2) + (t * v * cos(theta + w * t))/w;
        v_mat(2,1) = (cos(theta) - cos(theta + w * t))/w;
        v_mat(2,2) = -1 * (v * (cos(theta) - cos(theta + w * t))/w^2) + (t * v * sin(theta + w * t))/w;
    end
end

function m = calculate_M(u, alpha)
    v = u(1);
    w = u(2);
    
    m = zeros(2, 2);
    m(1, 1) = alpha(1) * v^2 + alpha(2) * w^2 ;
    m(2, 2) =  alpha(3) * v^2 + alpha(3) * w^2 ;
end

function [x_prime, y_prime, theta_prime] = find_x_prime(state, u, t)
    %% Function that calculates the expected x' of a robot after u
    % u is expected to be a vecotr, [v_ti, w_ti]
    % state is [x_0, y_0, theta_0]
    v = u(1);
    w = u(2);
    
    x_0 = state(1);
    y_0 = state(2);
    theta_0 = state(3);
    
    arc_length = v * t;
    
    if w ~= 0
        angle = w * t;
        radius = v/w;

        % Notes say 
        x_c = x_0 - radius * sin(theta_0);
        y_c = y_0 + radius * cos(theta_0);

        x_prime = x_c + radius * sin(theta_0 + w * t);
        y_prime = y_c - radius * cos(theta_0 + w * t);

        theta_prime = w * t + theta_0;
    else
        x_prime = x_0 + arc_length * cos(theta_0);
        y_prime = y_0 + arc_length * sin(theta_0);
        theta_prime = theta_0;
    end
   
end

function [x_prime, y_prime, theta_prime] = sample_motion_model_velocity(u, state, t, alpha)
    v = u(1) ;
    w = u(2);
    
    x_0 = state(1);
    y_0 = state(2);
    theta_0 = state(3);
    
    v_hat = v + sample_from_normal(alpha(1) * v^2 + alpha(2) * w^2);
    w_hat = w + sample_from_normal(alpha(3) * v^2 + alpha(4) * w^2);
    if w_hat ~= 0
        r = v_hat/w_hat;

        d_theta = w_hat * t;

        gamma = sample_from_normal(alpha(5) * v^2 + alpha(6)* w^2);

        x_prime = x_0 - r*sin(theta_0) + r * sin(theta_0 + d_theta);
        y_prime = y_0 + r*cos(theta_0) - r * cos(theta_0 + d_theta);
        theta_prime = theta_0 + d_theta + gamma * t;
    else
        x_prime = x_0 + v_hat * t * cos(theta_0);
        y_prime = y_0 + v_hat * t * sin(theta_0);
        theta_prime = theta_0;
    end
    
end

function [sample] = sample_from_normal(variance)
       all_rand = (2 * variance) .* rand(12, 1) - variance ;
       sample = sum(all_rand)/2;
end

function [] = plot_error_elipse(state, covariance, color)
%% Plot the error elipse for a covariance matrix....
% This code was taken from: https://www.visiondummy.com/2014/04/draw-error-ellipse-representing-covariance-matrix/
% This code is not mine! 
X0=state(1);
Y0=state(2);

% Calculate the eigenvectors and eigenvalues
[eigenvec, eigenval ] = eig(covariance);

% Get the index of the largest eigenvector
[largest_eigenvec_ind_c, r] = find(eigenval == max(max(eigenval)));
largest_eigenvec = eigenvec(:, largest_eigenvec_ind_c);

% Get the largest eigenvalue
largest_eigenval = max(max(eigenval));

% Get the smallest eigenvector and eigenvalue
if(largest_eigenvec_ind_c == 1)
    smallest_eigenval = max(eigenval(:,2))
    smallest_eigenvec = eigenvec(:,2);
else
    smallest_eigenval = max(eigenval(:,1))
    smallest_eigenvec = eigenvec(1,:);
end

% Calculate the angle between the x-axis and the largest eigenvector
angle = atan2(largest_eigenvec(2), largest_eigenvec(1));

% This angle is between -pi and pi.
% Let's shift it such that the angle is between 0 and 2pi
if(angle < 0)
    angle = angle + 2*pi;
end

% Get the 95% confidence interval error ellipse
chisquare_val = 2.4477;
theta_grid = linspace(0,2*pi);
phi = angle;
a=chisquare_val*sqrt(largest_eigenval);
b=chisquare_val*sqrt(smallest_eigenval);

% the ellipse in x and y coordinates 
ellipse_x_r  = a*cos( theta_grid );
ellipse_y_r  = b*sin( theta_grid );

%Define a rotation matrix
R = [ cos(phi) sin(phi); -sin(phi) cos(phi) ];

%let's rotate the ellipse to some angle phi
r_ellipse = [ellipse_x_r;ellipse_y_r]' * R;

% Draw the error ellipse
plot(r_ellipse(:,1) + X0,r_ellipse(:,2) + Y0,'-')
patch(r_ellipse(:,1) + X0,r_ellipse(:,2) + Y0, color)
hold on;

% Plot the eigenvectors
quiver(X0, Y0, largest_eigenvec(1)*sqrt(largest_eigenval), largest_eigenvec(2)*sqrt(largest_eigenval), '-m', 'LineWidth',2);
quiver(X0, Y0, smallest_eigenvec(1)*sqrt(smallest_eigenval), smallest_eigenvec(2)*sqrt(smallest_eigenval), '-g', 'LineWidth',2);
hold on;

% Set the axis labels
hXLabel = xlabel('x');
hYLabel = ylabel('y');

end


