%{
Eric Miller
Edm54
EECS 499
HW 2
P2) Odometry Motion Model
%}

%% 2a --> No noise

% Find the state change after U1 is applied

state = [2, 0, pi/2];
state_bar_0 = [1, 0, 0];
state_bar_1 = [3, -1, -1.571];
state_bar_2 = [3, -2, 0];

[x_one, y_one, theta_one] = s_prime_odometry(state_bar_0, state_bar_1, state);
state_1 = [x_one, y_one, theta_one]


[x_two, y_two, theta_two] = s_prime_odometry(state_bar_1, state_bar_2, state_1);
state_2 = [x_two, y_two, theta_two]

figure
hold on 
plot(state(1), state(2), 'r*')
plot(state_1(1), state_1(2), 'r*')
plot(state_2(1), state_2(2), 'r*')
text(state(1) + .10, state(2) + .1, 'State_0', 'FontSize', 15)
text(state_1(1) +.05, state_1(2) -.1, 'State_1', 'FontSize', 15)
text(state_2(1) -.25, state_2(2) - .1, 'State_2', 'FontSize', 15)
grid minor
plot([state(1), state_1(1)], [state(2), state_1(2)], '--k')
plot([state_1(1), state_2(1)], [state_1(2), state_2(2)], '--k')
title('Robot Motion, Perfect Exectution With Odometry Model')
ylabel('Y')
xlabel('X')

%% 2B --> sample_motion odometry
alpha = [.01, .002, .0001, .0001]
alpha1 = [.0001, .0002, .01, .0001]
state = [2, 0, pi/2];
state_bar_0 = [1, 0, 0];
state_bar_1 = [3, -1, -1.571];
state_bar_2 = [3, -2, 0];

[rot1, trans1, rot2] = get_deltas(state_bar_0, state_bar_1)
u1 = [rot1, trans1, rot2]

[rot1, trans1, rot2] = get_deltas(state_bar_1, state_bar_2)
u2 = [rot1, trans1, rot2]
x_err = [];
y_err = [];

x_err2 = [];
y_err2 = [];

for i= 1:1000
    [x_one, y_one, theta_one]  = sample_motion_odometry(u1, state, alpha);
    state_1 = [x_one, y_one, theta_one];
    x_err = [x_err x_one];
    y_err = [y_err y_one];
    
    [x_two, y_two, theta_two]  = sample_motion_odometry(u2, state_1, alpha);
    x_err2 = [x_err2 x_two];
    y_err2 = [y_err2 y_two];
end

figure
hold on 
scatter(x_err, y_err, 15,'filled', 'm')
scatter(x_err2, y_err2, 15,'filled', 'm')
plot(state(1), state(2), 'r*')
plot(state_1(1), state_1(2), 'r*')
plot(state_2(1), state_2(2), 'r*')
plot([state(1), state_1(1)], [state(2), state_1(2)], '--k', 'LineWidth', 1)
plot([state_1(1), state_2(1)], [state_1(2), state_2(2)], '--k',  'LineWidth',2)

grid minor
title('Robot Motion, With Error, \alpha = [.0001, .0002, .01, .0001]')
ylabel('Y')
xlabel('X')

%% 2C --> motion_model_odometry

state = [2, 0, pi/2];
state_bar_0 = [1, 0, 0];
state_bar_1 = [3, -1, -1.571];
[rot1, trans1, rot2] = get_deltas(state_bar_0, state_bar_1)
u1 = [rot1, trans1, rot2]
%alpha = [.01, .002, .0001, .0001]
alpha = [.0001, .0002, .01, .0001]

p = 0;
probs = [];
x_val = [];
y_val = [];
all_x_val = [];
all_probs = [];
for y = 1.5:.01:2.50
    probs = [];
    x_val = [];
    for x = 2.25:.01:3.75
        p = 0;
        for i = -2 * pi: .01 : 2 * pi
            state_prime = [x, y, i] ;
            state = [2, 0, pi/2];
            prob = motion_model_odometry(state_prime, u1, state, alpha);
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
figure
hold on
scatter(all_x_val, y_val, 40,  all_probs/sum(all_probs), 'filled')
title('Robot Motion, With Error, \alpha = [.01, .002, .0001, .0001]', 'FontSize', 14)
ylabel('Y', 'FontSize', 16)
xlabel('X', 'FontSize', 16)
%%

figure
hold on
scatter(all_x_val, y_val, 40,  all_probs/sum(all_probs), 'filled')
plot(state(1), state(2), 'r*')
plot(state_1(1), state_1(2), 'r*')
plot(state_2(1), state_2(2), 'r*')
plot([state(1), state_1(1)], [state(2), state_1(2)], '--k', 'LineWidth', 2)
plot([state_1(1), state_2(1)], [state_1(2), state_2(2)], '--k',  'LineWidth',2)

title('Robot Motion, With Error, \alpha = [.01, .002, .0001, .0001]', 'FontSize', 14)
ylabel('Y', 'FontSize', 16)
xlabel('X', 'FontSize', 16)
old = colormap('gray')
colormap(flipud(old))
colorbar


%%
function [x_prime, y_prime, theta_prime]  = s_prime_odometry(state_bar, state_bar_prime, state)
    x_bar_prime = state_bar_prime(1);
    y_bar_prime = state_bar_prime(2);
    theta_bar_prime = state_bar_prime(3);
    
    x_bar = state_bar(1);
    y_bar = state_bar(2);
    theta_bar = state_bar(3);
    
    x_0 = state(1);
    y_0 = state(2);
    theta_0 = state(3);
    
    delta_rot_1 = atan2(y_bar_prime - y_bar, x_bar_prime - x_bar) - theta_bar;
    delta_trans = sqrt((x_bar_prime - x_bar)^2 + (y_bar_prime - y_bar)^2);
    delta_rot_2 = theta_bar_prime - theta_bar - delta_rot_1;
    
    x_prime = x_0 + delta_trans * cos(theta_0 + delta_rot_1 );
    y_prime = y_0 + delta_trans * sin(theta_0 + delta_rot_1 );
    theta_prime = theta_0 + delta_rot_1 + delta_rot_2;
    
end

function [delta_rot_1, delta_trans, delta_rot_2] = get_deltas(state_bar, state_bar_prime)
    x_bar_prime = state_bar_prime(1);
    y_bar_prime = state_bar_prime(2);
    theta_bar_prime = state_bar_prime(3);
    
    x_bar = state_bar(1);
    y_bar = state_bar(2);
    theta_bar = state_bar(3);


    delta_rot_1 = atan2(y_bar_prime - y_bar, x_bar_prime - x_bar) - theta_bar;
    delta_trans = sqrt((x_bar_prime - x_bar)^2 + (y_bar_prime - y_bar)^2);
    delta_rot_2 = theta_bar_prime - theta_bar - delta_rot_1;

end


function [x_prime, y_prime, theta_prime]  = sample_motion_odometry(u, state, alpha)
   
    
    x_0 = state(1);
    y_0 = state(2);
    theta_0 = state(3);
    
    delta_rot_1 = u(1);
    delta_trans = u(2);
    delta_rot_2 = u(3);
    
    delta_rot_1_hat = delta_rot_1 + sample_from_normal(alpha(1) * abs(delta_rot_1)  + alpha(2) * abs(delta_trans));
    delta_trans_hat = delta_trans + sample_from_normal(alpha(3) * abs(delta_trans)  + alpha(4) * abs(delta_rot_1) * abs(delta_rot_2));
    delta_rot_2_hat = delta_rot_2 + sample_from_normal(alpha(1) * abs(delta_rot_2)  + alpha(2) * abs(delta_trans));
    
    x_prime = x_0 + delta_trans_hat * cos(theta_0 + delta_rot_1_hat );
    y_prime = y_0 + delta_trans_hat * sin(theta_0 + delta_rot_1_hat );
    theta_prime = theta_0 + delta_rot_1_hat + delta_rot_2_hat;
    
end



function p = motion_model_odometry(state_prime, u, state, alpha)
    x_0 = state(1);
    y_0 = state(2);
    theta_0 = state(3);
    
    x_prime = state_prime(1);
    y_prime = state_prime(2);
    theta_prime = state_prime(3);
   
    delta_rot_1 = u(1);
    delta_trans = u(2);
    delta_rot_2 = u(3);
    
    delta_rot_1_hat = atan2(y_prime - y_0, x_prime - x_0) - theta_0;
    delta_trans_hat = sqrt((x_prime - x_0)^2 + (y_prime - y_0)^2);
    delta_rot_2_hat = theta_prime - theta_0 - delta_rot_1_hat;
    
    p1 = prob_normal_distribution(delta_rot_1 - delta_rot_1_hat, alpha(1) * delta_rot_1_hat^2 + alpha(2) * delta_trans_hat^2);
    
    p2 = prob_normal_distribution(delta_trans - delta_trans_hat, alpha(3) * delta_trans_hat^2 + alpha(4) * (delta_rot_1_hat^2 + delta_rot_2_hat^2));
    
    p3 = prob_normal_distribution(delta_rot_2 - delta_rot_2_hat, alpha(1) * delta_rot_2_hat^2 + alpha(2) * delta_trans_hat^2);
    
    p = p1 * p2 * p3;
    
end

function prob = prob_normal_distribution(value, variance)
    
    prob = (1/sqrt(2 * pi * variance)) * exp(-.5 * value^2 / variance);

end




function [sample] = sample_from_normal(variance)
       all_rand = (2 * variance) .* rand(12, 1) - variance ;
       sample = sum(all_rand)/2;
    end







