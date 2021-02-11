%{
Eric Miller
Edm54
EECS 499
HW 2
P3) Beam-Based Measurement Model
%}
t = 1
max_range = 5
state = [2, 0, pi/2];
sigma_hit = .05;
lambda_short = .05;

% hit, short, max, rand;
z_arr = [.8, .05, .05, .1];

u1 = [.5, 0];
u2 = [.2, .76];
wall_x = 0:.1:3;
wall_y = zeros(length(0:.1:3)) + 3;

[x_prime, y_prime, theta_prime] = find_x_prime(state, u1, t);
state_1 = [x_prime, y_prime, theta_prime] 
true_range_1 = calc_true_range(state_1)

[x_prime, y_prime, theta_prime] = find_x_prime(state_1, u2, t);
state_2 = [x_prime, y_prime, theta_prime] 
true_range_2 = calc_true_range(state_2)

%%
u1 = [.5, 0];
u2 = [.2, .76];
state = [2, 0, pi/2];
s0 = state;
x = [];
y = [];
t = .001;
for i = 0:t:1-t
    [x_one, y_one, theta_one] = find_x_prime(s0, u1, t);
    s0 = [x_one, y_one, theta_one];
    x = [x x_one];
    y = [y y_one];
end
s1 = s0
for i = 0:t:1-t
    [x_two, y_two, theta_two] = find_x_prime(s1, u2, t);
    s1 = [x_two, y_two, theta_two];
    x = [x x_two];
    y = [y y_two];
end
s2 = s1

figure
scatter(x,y)
%%
probs = []
hit = []
short = []
max = []
rand = []
xs = 0:.0001:6

for x = xs
    
    [prob, hit1, short1, max1, random1] = prob_measurement(x, true_range_1, max_range, sigma_hit, lambda_short, z_arr);
    probs = [probs prob];
   
    hit = [hit hit1];
    short = [short short1];
    max = [max max1];
    rand = [rand random1];
  
    
end
%%
probs2 = []
hit2 = []
short2 = []
max2 = []
rand2 = []
xs = 0:.0001:6

for x = xs
    
    [prob, hit1, short1, max1, random1] = prob_measurement(x, true_range_2, max_range, sigma_hit, lambda_short, z_arr);
    probs2 = [probs2 prob];
   
    hit2 = [hit2 hit1];
    short2 = [short2 short1];
    max2 = [max2 max1];
    rand2 = [rand2 random1];
  
    
end
%%
figure
hold on
plot(xs, short2/.05)
xlim([0, 5])
title('P_{short} for z_2')
grid minor
ylabel('P_{short}')
xlabel('z_2')
%%
figure
hold on
plot(xs, rand2/.1)
xlim([0, 5])
title('P_{rand} for z_2')
grid minor
ylabel('P_{rand}')
xlabel('z_2')
ylim([0, 1])
%%
figure
hold on
plot(xs, max2/.05)
title('P_{max} for z_2')
grid minor
ylabel('P_{max}')
xlabel('z_2')
%%
figure
hold on
plot(xs, hit2/.8)
title('P_{hit} for z_2')
grid minor
ylabel('P_{hit}')
xlabel('z_2')

%% Plot PDF of Both ranges

figure
hold on
scatter(xs, probs, 10, 'filled' )
xline(true_range_1, '--k')
ht = text(true_range_1 - .1, .5, strcat('True Range= ', num2str(true_range_1)), 'FontSize', 14)
set(ht, 'Rotation', 90)
title('Probability Density Function, P(z_1 \mid x_1, {\it m})')
grid minor
ylabel('P(z_1 \mid x_1, {\it m})')

xlabel('z_1')

%%
figure
hold on
scatter(xs, probs2, 10, 'filled' )
xline(true_range_2, '--k')
ht = text(true_range_2 - .1, .5, strcat('True Range= ', num2str(true_range_2)), 'FontSize', 14)
set(ht, 'Rotation', 90)
title('Probability Density Function, P(z_2 \mid x_2, {\it m})')
grid minor
ylabel('P(z_2 \mid x_2, {\it m})')

xlabel('z_2')

%%

figure
hold on
scatter(xs, hit2, 10, 'filled' )
xlim([0, 5])

%% Plot Robot Motion

figure
hold on 
plot(state(1), state(2), 'r*', 'LineWidth', 7)
plot(state_1(1), state_1(2), 'r*', 'LineWidth', 7)
plot(state_2(1), state_2(2), 'r*', 'LineWidth', 7)
plot(wall_x, wall_y, 'k', 'LineWidth', 7)
text(state(1) +.1, state(2) + .15, 'State_0', 'FontSize', 14)
text(state_1(1) + .1, state_1(2) -.1, 'State_1', 'FontSize', 14)
text(state_2(1) +.175, state_2(2), 'State_2', 'FontSize', 14)
plot( [state_1(1), state_1(1)], [state_1(2), 3], '--k', 'LineWidth', 1)
plot( [state_2(1), state_2(1) - 3.441], [state_2(2), state_2(2) + 3.6242], '--k', 'LineWidth', 1)
scatter(x,y, 5, 'filled')
ht = text(state_1(1) + .1 , 1, strcat('True Range= ', num2str(true_range_1)), 'FontSize', 12)
ht1 = text(state_2(1) - 1 , 2, strcat('True Range= ', num2str(true_range_2)), 'FontSize', 12)

set(ht, 'Rotation', 90)
set(ht1, 'Rotation', -35)
title('Movement of Robot and True Range to the Wall')
xlabel('X')
ylabel('Y')
% xlim([-1, 3.5])
%%
function [prob, p1, p2, p3, p4] = prob_measurement(z, z_exp, z_max, sigma_hit, lambda_short, z_arr)
    p1 = prob_hit(z, z_exp, sigma_hit, z_max) * z_arr(1);
    p2 = prob_short(z, z_exp, lambda_short)  * z_arr(2);
    p3 = failure(z, z_max) * z_arr(3);
    p4 = random(z, z_max) * z_arr(4);
    prob = p1 + p2 + p3 + p4;
end


function prob = prob_hit(z, z_exp, variance, z_max) 

    % Estimate CDF for normalizing factor
     eta = .5 * (1 + erf((z_max - z_exp)/(variance * sqrt(2))));
     eta = 1/eta;

    if z <=z_max
        prob = eta * prob_normal_distribution(z-z_exp, variance);
    else
        prob = 0;
    end
end

function prob = prob_normal_distribution(value, variance)
    
    prob = (1/sqrt(2 * pi * variance)) * exp(-.5 * value^2 / variance);

end

function prob = prob_short(z, z_exp, lambda) 
    
    eta = 1 / (1 - exp(-lambda * z_exp));
    
    if z <=z_exp
        prob = eta * lambda * exp(-lambda * z);
    else
        prob = 0;
    end

end

function prob = random(z, z_max) 
    eta = 1;
    if z > z_max
        prob = 0;
    else
        prob =  eta * 1/z_max;
    end
    

end

function prob = failure(z, z_max) 
    if z == z_max
        prob = 1;
    else
        prob = 0;
    end

end

function [r] = calc_true_range(state)
    

    if state(1) > 3 
        y = sqrt((state(1) - 3)^2 + (state(2) - 3)^2);
    elseif state(1) < 0
        y = sqrt((state(1) - 0)^2 + (state(2) - 3)^2);
    else
        y = abs(state(2) - 3);
    end
   
    if abs(state(3) - pi/2) < .01 
        r = y;
    else
        x =state(1)-  y * tan( state(3) - pi/2)
        
        if x > 3 || x < 0
            r = 5;
            y = r * cos(state(3) - pi/2) + state(2)
            x = state(1)- sqrt(r^2 - y^2)
        else
            r = y / cos(state(3) - pi/2);
        end
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
    
    if w > 0
        arc_length = v * t;
        angle = w * t;

        radius = v/w;

        % Notes say 
        x_c = x_0 - radius * sin(theta_0);
        y_c = y_0 + radius * cos(theta_0);

        x_prime = x_c + radius * sin(theta_0 + w * t);
        y_prime = y_c - radius * cos(theta_0 + w * t);

        theta_prime = w * t + theta_0;
    else
        theta_prime = w * t + theta_0;
        x_prime = x_0 + cos(theta_0) * v * t
        y_prime = y_0 + sin(theta_0) * v * t        
    end
end

