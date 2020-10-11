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
state_0 = [2, 2, 0];
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
%% Particle Filter Algorithm
m = 1000
sample_set = []
weight_set = []
% Create original samples
low_ind = 1
% figure
% hold on

for state_index = low_ind:8
    sample_set = []
    weight_set = []
    for i = 1:m
        [x1,y1,theta1] = sample_motion_model_velocity(u(state_index,:), x(state_index ,:), t, alpha);
        sample = [x1,y1,theta1];
        weight = landmark_detection_model(z(state_index,1:2), sample, map(z(state_index, 3),:), variance_r, variance_phi);
        sample_set = [sample_set ;sample];
        weight_set = [weight_set ;weight];
    end
    %%
    weight_sum = sum(weight_set);
    probability_set = weight_set/weight_sum;
    resampled_set = [];
    % Resample using weights
    for i = 1:m
        resampled_sample =  sample_set(find(rand<cumsum(probability_set),1,'first'), :);
        resampled_set = [resampled_set;resampled_sample];
    end
    
    for i = 1:m
        plot(sample_set(i, 1), sample_set(i, 2), 'r*')
        plot(resampled_set(i, 1), resampled_set(i, 2), 'b*')
    end
    
end  


%%

figure
hold on
for i = 1:m
    plot(sample_set(i, 1), sample_set(i, 2), 'r*')
    plot(resampled_set(i, 1), resampled_set(i, 2), 'b*')
end  

%%
x = []
y = []
for i = 1:m
    x = [x sample_set(i, 1)];
    y = [y sample_set(i, 2)];   
end

figure 
scatter(x, y, 5, probability_set)
colorbar


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

function prob = prob_normal_distribution(value, variance)
    
    prob = (1/sqrt(2 * pi * variance)) * exp(-.5 * value^2 / variance);

end

function [prob] = landmark_detection_model(z, state, m, sigma_r, sigma_phi)
   % measured range
   measured_range = z(1);
   
   % bearing
   measured_bearing = z(2);
        
   robot_x = state(1);
   robot_y = state(2);
   
   landmark_x = m(1);
   landmark_y = m(2);
   
   true_range = sqrt((landmark_x - robot_x)^2 + (landmark_y - robot_y)^2);
   true_bearing = atan2(landmark_y - robot_y, landmark_x - robot_x);
   if true_bearing <0
       true_bearing = 2*pi + true_bearing ;
   end
   
   prob_range = prob_normal_distribution(measured_range - true_range, sigma_r);
   prob_bearing = prob_normal_distribution(measured_bearing - true_bearing, sigma_phi);
   prob_bearing2 = prob_normal_distribution(measured_bearing - (true_bearing + 2*pi), sigma_phi);
   
   p_max = max(prob_bearing, prob_bearing2)
   
   if p_max == prob_bearing2
        disp("!!!!!!!!")
   end
       
       
   prob = prob_range * p_max;
        
end

