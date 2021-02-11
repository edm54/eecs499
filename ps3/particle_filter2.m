%{
Eric Miller
Edm54
EECS 499
Project 1
Particle Filter
%}

%% initialize parameters
alpha = [.0001, .0001, .01,.0001, .0001,.0001];
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
% Plot Landmarks
scatter(map(:,1), map(:,2), 100, 'filled', 'k')

for i = 1:8
    [x1, y1, theta_1] = find_x_prime(state, u(i, :), t);
    state = [x1, y1, theta_1];
    x = [x ;state];
end
% Plot Noiseless trajectory
scatter(x(:,1), x(:,2), 'filled', 'k')

%% Particle Filter Algorithm
num_particles = 10000 
sample_set = []
weight_set = []

% Create original samples
low_ind = 1
x_set = x

% Initialize particle set
resampled_set = []
for i = 1:num_particles
    resampled_set = [resampled_set; x_set(1,:)];
end

w_mat1 = []
s_mat1 = []
s_mat2 = {}
m_first = []
m_second = []
figure
hold on
% tiledlayout(2,4)

for state_index = low_ind:8
    sample_set = []
    weight_set = []
    landmark_xy = z(state_index,1:2);
    landmark_id = z(state_index, 3);
    action = u(state_index,:);
    for i = 1:num_particles
        % Sample from motion model with orignal state as each resampled  particle
        [x1,y1,theta1] = sample_motion_model_velocity(action, resampled_set(i ,:), t, alpha);
        sample = [x1,y1,theta1]; 
        weight = landmark_detection_model(landmark_xy, sample, map(landmark_id,:), variance_r, variance_phi);
        
        sample_set = [sample_set ;sample weight];
        weight_set = [weight_set ;weight];
    end
    w_mat1 = [w_mat1 ;weight_set'];
    %%
    weight_sum = sum(weight_set);
    m_first = [ m_first mean(weight_set)];
    probability_set = weight_set/weight_sum;
    resampled_set = [];
    
    % Resample using weights
    for i = 1:num_particles
        resampled_sample =  sample_set(find(rand<cumsum(probability_set),1,'first'), :);
        resampled_set = [resampled_set;resampled_sample];
    end
    m_second = [m_second mean(resampled_set(:, 4))];
    
    scatter(sample_set(:, 1), sample_set(:, 2), 1.5, 'r', 'filled')
    scatter(resampled_set(:, 1), resampled_set(:, 2), 15, 'b', 'filled' )    
    
    % For creating the tiled plot
%     nexttile
%     hold on
%     scatter(sample_set(:, 1), sample_set(:, 2),  5, 'r', 'filled')
%     scatter(resampled_set(:, 1), resampled_set(:, 2), 5, 'b', 'filled' )    
    xlabel('X')
    ylabel('Y')
%     title(['Action: ',num2str(state_index)])
%     plot(x(state_index+1,1), x(state_index+1,2), '*m')
    grid minor
end  

%% Plot the fractions
figure
hold on
bar(m_second./m_first)
title('Ratio of Average Weight After Resampling to Average Weight Before Resampling')
xlabel('Action')
ylabel('Ratio')
grid minor
text(1:length(m_second./m_first),m_second./m_first,num2str((m_second./m_first)'),'vert','bottom','horiz','center'); 
box off
title('Particle Filter Results')
xlabel('X'), 
ylabel('Y')
% legend('Perfect Execution', 'Before Landmark', 'After Landmark')

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

function [prob] = landmark_detection_model(z, state, m, variance_r, variance_phi)
   % measured range
   measured_range = z(1);
   
   % bearing
   measured_bearing = z(2);
        
   robot_x = state(1);
   robot_y = state(2);
   robot_theta = state(3);
   
   landmark_x = m(1);
   landmark_y = m(2);
   
   true_range = sqrt((landmark_x - robot_x)^2 + (landmark_y - robot_y)^2);
   true_bearing = atan2(landmark_y - robot_y, landmark_x - robot_x) - robot_theta;
%    if true_bearing <0
%        true_bearing = 2*pi + true_bearing ;
%    end
    
   range_err =  measured_range - true_range;

   prob_range = prob_normal_distribution(range_err, variance_r);
   prob_bearing = prob_normal_distribution(measured_bearing - true_bearing, variance_phi);
   prob_bearing2 = prob_normal_distribution(measured_bearing - (true_bearing + 2*pi), variance_phi);
   
   p_max = max(prob_bearing, prob_bearing2);       
       
   prob = prob_range * p_max;
        
end

function delta_r = true_range_delta(z, state, m)

   measured_range = z(1);
   
   robot_x = state(1);
   robot_y = state(2);
   
   landmark_x = m(1);
   landmark_y = m(2);
   tr = sqrt((landmark_x - robot_x)^2 + (landmark_y - robot_y)^2);
    
   delta_r = abs(tr - measured_range);
   
end

  
function delta_b = true_bearing_delta(z, state, m)

   measured_bearing = z(2);
   
   robot_x = state(1);
   robot_y = state(2);
   robot_theta = state(3);
    
   landmark_x = m(1);
   landmark_y = m(2);
   
   true_bearing = atan2(landmark_y - robot_y, landmark_x - robot_x)   - robot_theta;
   if true_bearing <0
       true_bearing = 2*pi + true_bearing ;
   end

    
   delta_b = abs(true_bearing - measured_bearing);
   
end
