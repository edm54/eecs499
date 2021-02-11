%{
Eric Miller
Edm54
EECS 499
HW 2
P4) Landmark Based Model
%}


m1 = [1, 1];
m2 = [2, 1];
m3 = [3, 2];
m4 = [2, 4];

sigma_r = .3;
sigma_phi = .5;


variance_r = sigma_r^2;
variance_phi = sigma_phi^2;


state_0 = [2, 0, pi/2];

z0 = [1.6213, 2.430; 1.1270, 1.4140];
z1 = [.51, -2.68; 1.027, 1.405];

p1_1 = landmark_detection_model(z0(1, :), state_0, m1, variance_r, variance_phi)
p1_2 = landmark_detection_model(z0(2, :), state_0, m2, variance_r, variance_phi)
p_z0 = p1_1  * p1_2

u1 = [1.2, -1.0];
t = 1;
[x_one, y_one, theta_one] = find_x_prime(state_0, u1, t);
state_1 = [x_one, y_one, theta_one];

p2_1 = landmark_detection_model(z1(1, :), state_1, m2, variance_r, variance_phi)
p2_2 = landmark_detection_model(z1(2, :), state_1, m3, variance_r, variance_phi)
p_z1 = p2_1  * p2_2
%%
all_probs =[]
bearing = 1:.01:3.5
range = .5:.01:2.2
ranges = []
bearings = []
for b = bearing
    for r = range
        z = [r, b];
        p = landmark_detection_model(z, state_0, m1, sigma_r, sigma_phi);
        all_probs = [all_probs p];
        ranges = [ranges r];
        bearings = [bearings b];
    end
end
%%
figure
hold on 
scatter(ranges, bearings,  16, all_probs, 'filled')
colorbar
ylim([1, 3.5])
xlim([.5, 2.2])
h = plot(1.4142, 2.356, 'r*', 'LineWidth', 7)
xlabel('Range')
ylabel('Bearing')
grid minor
legend([h], 'Actual Range and Bearing')
title('Probability of a measurement, P(z_0^1|x_0,m)')
%%
s0 = [2, 0, pi/2];
u1 = [1.2, -1.0];
x = [];
y = [];
t = .001;
for i = 0:t:1-t
    [x_one, y_one, theta_one] = find_x_prime(s0, u1, t);
    s0 = [x_one, y_one, theta_one];
    x = [x x_one];
    y = [y y_one];
end
%%
figure
hold on
plot(m1(1), m1(2), 'r*', 'LineWidth', 7)
plot(m2(1), m2(2), 'r*', 'LineWidth', 7)
plot(m3(1), m3(2), 'r*', 'LineWidth', 7)
plot(m4(1), m4(2), 'r*', 'LineWidth', 7)
plot(state_0(1), state_0(2), 'b*', 'LineWidth', 7)
plot(state_1(1), state_1(2), 'b*', 'LineWidth', 7)
plot( [state_0(1), m1(1)], [state_0(2), m1(2)], '--k', 'LineWidth', 1)
plot( [state_0(1), m2(1)], [state_0(2), m2(2)], '--k', 'LineWidth', 1)
plot( [state_1(1), m3(1)], [state_1(2), m3(2)], '--k', 'LineWidth', 1)
plot( [state_1(1), m2(1)], [state_1(2), m2(2)], '--k', 'LineWidth', 1)
scatter(x,y, 5, 'filled')
ht = text(state_0(1) - .5 , 1, strcat('P(z_0|x_0, m) = ', num2str(p_z0)), 'FontSize', 14)
ht1 = text(state_1(1) - .25 , 1.5, strcat('P(z_1|x_1, m) = ', num2str(p_z1)), 'FontSize', 14)
text(state_0(1) - .5 , .65,'z_0^1', 'FontSize', 14)
text(state_0(1) - .05 , .65,'z_0^2', 'FontSize', 14)
text(state_0(1) + .25 , 1.15,'z_1^2', 'FontSize', 14)
text(state_1(1) + .25 , 1.45,'z_1^3', 'FontSize', 14)
grid minor
title('Probability of Sensor Measurements, z_0 & z_1')
xlabel('X')
ylabel('Y')

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
   
   prob_range = prob_normal_distribution(measured_range - true_range, sigma_r);
   prob_bearing = prob_normal_distribution(measured_bearing - true_bearing, sigma_phi);
   
   prob = prob_range * prob_bearing;
        
end


function prob = prob_normal_distribution(value, variance)
    
    prob = (1/sqrt(2 * pi * variance)) * exp(-.5 * value^2 / variance);

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
    angle = w * t;
    
    radius = v/w;
    
    % Notes say 
    x_c = x_0 - radius * sin(theta_0);
    y_c = y_0 + radius * cos(theta_0);
    
    x_prime = x_c + radius * sin(theta_0 + w * t);
    y_prime = y_c - radius * cos(theta_0 + w * t);
    
    theta_prime = w * t + theta_0;

end




