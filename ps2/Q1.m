%{
Eric Miller
Edm54
EECS 499
HW 2
P1) Velocity Motion Model
%}

%% 1A) 
% Find the state change after U1 is applied

state = [2, 0, pi/2];
t = 1;
u = [pi/2, pi/2];

[x_one, y_one, theta_one] = find_x_prime(state, u, t);
state_1 = [x_one, y_one, theta_one]

u1 = [pi/2, -pi/2];
[x_two, y_two, theta_two] = find_x_prime(state_1, u1, t);
state_2 = [x_two, y_two, theta_two]

figure
hold on 
plot(state(1), state(2), 'r*')
plot(state_1(1), state_1(2), 'r*')
plot(state_2(1), state_2(2), 'r*')
text(state(1) -.25, state(2) + .1, 'State_0')
text(state_1(1) -.1, state_1(2) -.1, 'State_1')
text(state_2(1) -.1, state_2(2) + .1, 'State_1')

%% Fill in intermediate states for a nice plot
state = [2, 0, pi/2];
s0 = state;
x = [];
y = [];
t = .01;
for i = 0:t:1-t
    [x_one, y_one, theta_one] = find_x_prime(state, u, t);
    state = [x_one, y_one, theta_one];
    x = [x x_one];
    y = [y y_one];
end
state_1 = state
for i = 0:t:1-t
    [x_two, y_two, theta_two] = find_x_prime(state, u1, t);
    state = [x_two, y_two, theta_two];
    x = [x x_two];
    y = [y y_two];
end
state_2 = state

figure
hold on
title('Robot Motion, Perfect Exectution')
scatter(x,y, 'filled')
plot(s0(1), s0(2), '-s', 'MarkerSize', 10, 'MarkerEdgeColor','red',...
    'MarkerFaceColor','red')
plot(state_1(1), state_1(2), '-s', 'MarkerSize', 10, 'MarkerEdgeColor','red',...
    'MarkerFaceColor','red')
plot(state_2(1), state_2(2), '-s', 'MarkerSize', 10, 'MarkerEdgeColor','red',...
    'MarkerFaceColor','red')
text(s0(1) -.15, s0(2) + .1, 'State_0', 'FontSize',14)
text(state_1(1) -.05, state_1(2) -.1, 'State_1', 'FontSize',14)
text(state_2(1) +.05, state_2(2) - .05, 'State_2', 'FontSize',14)
grid minor

%% 1B --> Sample_motion_model_velocity
state = [2, 0, pi/2];
t = 1;
u = [pi/2, pi/2];
alpha1 = [.005, .005, .0001, .0001, .0001, .0001]
alpha = [.0001, .0001, .01, .0001, .0001, .0001]
% alpha = [0, 0 , 0, 0, 0 , 0]

x_err = []
y_err = []
x_err2 = []
y_err2 = []
for i= 1:1000
    [x_one, y_one, theta_one]  = sample_motion_model_velocity(u, state, t, alpha);
    state_1 = [x_one, y_one, theta_one];
    x_err = [x_err x_one];
    y_err = [y_err y_one];
    
    [x_two, y_two, theta_two]  = sample_motion_model_velocity(u1, state_1, t, alpha);
    x_err2 = [x_err2 x_two];
    y_err2 = [y_err2 y_two];
end

state = [2, 0, pi/2];
s0 = state;
x = [];
y = [];
t = .01;
for i = 0:t:1-t
    [x_one, y_one, theta_one] = find_x_prime(state, u, t);
    state = [x_one, y_one, theta_one];
    x = [x x_one];
    y = [y y_one];
end
state_1 = state
for i = 0:t:1-t
    [x_two, y_two, theta_two] = find_x_prime(state, u1, t);
    state = [x_two, y_two, theta_two];
    x = [x x_two];
    y = [y y_two];
end
state_2 = state

figure

hold on
scatter(x,y, 'filled')
% figure
scatter(x_err, y_err, 10,'filled')
scatter(x_err2, y_err2, 10,'filled')
title('Robot Motion, With Error, \alpha = [.0001, .0001, .01, .0001, .0001, .0001]')
ylabel('Y')
xlabel('X')
grid minor
legend('Noiseless Trajectory')
%% 1C --> motion_model_velocity
state_prime = [1, 1, pi/2] ;
state = [2, 0, pi/2];
t = 1;
u = [pi/2, pi/2];
alpha = [.0001, .0001, .01, .0001, .0001, .0001]
% alpha = [.005, .005, .0001, .0001, .0001, .0001]
motion_model_velocity(state_prime, u, state, t, alpha)

p = 0;
probs = [];
x_val = [];
y_val = [];
all_x_val = [];
all_probs = [];
for y = .6:.005: 1.20
    probs = [];
    x_val = [];
    for x = .8:.005: 1.20
        p = 0;
        for i = -2 * pi: .01 : 2 * pi
            state_prime = [x, y, i] ;
            state = [2, 0, pi/2];
            prob = motion_model_velocity(state_prime, u, state, t, alpha);
            p = p + prob;
        end
        probs = [probs p];
        x_val = [x_val x];
        y_val = [y_val y];
    end
    all_probs = [all_probs probs];
    all_x_val = [all_x_val x_val];
    
end
%%
state = [2, 0, pi/2];
s0 = state;
x = [];
y = [];
t = .01;
for i = 0:t:1-t
    [x_one, y_one, theta_one] = find_x_prime(state, u, t);
    state = [x_one, y_one, theta_one];
    x = [x x_one];
    y = [y y_one];
end

%%
figure
scatter(x,y, 'filled')
%%

figure
hold on
scatter(all_x_val, y_val, 40,  all_probs/sum(all_probs), 'filled')
p = scatter(x,y, 20, 'filled')
ylim([0, 1.25])
xlim([.75, 2])
title('Robot Motion, With Error, \alpha = [.0001, .0001, .01, .0001, .0001, .0001]', 'FontSize', 10)
ylabel('Y', 'FontSize', 16)
xlabel('X', 'FontSize', 16)
% grid minor
legend([p], 'Noiseless Trajectory')
old = colormap('gray')
colormap(flipud(old))
colorbar
%%
figure
scatter(x_val, probs/sum(probs))


%%
function p = motion_model_velocity(state_prime, u, state, t, alpha)
    x_prime = state_prime(1);
    y_prime = state_prime(2);
    theta_prime = state_prime(3);
    
    x_0 = state(1);
    y_0 = state(2);
    theta_0 = state(3);
    
    v = u(1) ;
    w = u(2);
    
    mu = .5 * ((x_0 - x_prime)*cos(theta_0) + (y_0 - y_prime)*sin(theta_0))/((y_0 - y_prime)*cos(theta_0) - (x_0 - x_prime)*sin(theta_0));
    x_star = (x_0 + x_prime)/2  + mu * (y_0 - y_prime);
    y_star = (y_0 + y_prime)/2  + mu * (x_prime - x_0);
    
    r_star = ((x_0 - x_star)^2 + (y_0 - y_star)^2)^.5;
    
    delta_theta = atan2(y_prime - y_star, x_prime - x_star) - atan2(y_0 - y_star, x_0 - x_star);
    
    v_hat = r_star * delta_theta / t ;
    w_hat = delta_theta / t ;
    gamma_hat = (theta_prime - theta_0)/t - w_hat  ;
    
    p1 = prob_normal_distribution(v - v_hat, alpha(1) * abs(v) + alpha(2) * abs(w));
    p2 = prob_normal_distribution(w - w_hat, alpha(3) * abs(v) + alpha(4) * abs(w));
    p3 = prob_normal_distribution(gamma_hat, alpha(5) * abs(v) + alpha(6) * abs(w));
    
    p = p1 * p2 * p3;
    
end
%%
function prob = prob_normal_distribution(value, variance)
    
    prob = (1/sqrt(2 * pi * variance)) * exp(-.5 * value^2 / variance);

end




%%
function [x_prime, y_prime, theta_prime] = sample_motion_model_velocity(u, state, t, alpha)
    v = u(1) ;
    w = u(2);
    
    x_0 = state(1);
    y_0 = state(2);
    theta_0 = state(3);
    
    v_hat = v + sample_from_normal(alpha(1) * v^2 + alpha(2) * w^2);
    w_hat = w + sample_from_normal(alpha(3) * v^2 + alpha(4) * w^2);
    
    r = v_hat/w_hat;
    
    d_theta = w_hat * t;
    
    gamma = sample_from_normal(alpha(5) * v^2 + alpha(6)* w^2);
       

    x_prime = x_0 - r*sin(theta_0) + r * sin(theta_0 + d_theta);
    y_prime = y_0 + r*cos(theta_0) - r * cos(theta_0 + d_theta);
    
    theta_prime = theta_0 + d_theta + gamma * t;
end


%%
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

function [sample] = sample_from_normal(variance)
       all_rand = (2 * variance) .* rand(12, 1) - variance ;
       sample = sum(all_rand)/2;
end



%%
