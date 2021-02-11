%{
Eric Miller
Edm54
EECS 499

P5) Gaussian with non-linear transformation

This file holds the code used to generate the plots for P5. 
Note that most of this code is for generating plots, as the normal
distribution algorithm is in the function at the bottom. 

%}



%% Generate N samples from normal and plot
x = sample_n_from_normal_shift(.5, 1000, 1);
figure
histogram(x, 1000, 'FaceColor', 'b', 'EdgeColor', 'b')
xlim([-1, 3])
[f,xi] = ksdensity(x);
hold on
ylabel('P(x)')
xlabel('X')
yyaxis right
plot(xi,f, 'r');
ylabel('PDF')
title('N(1, .5), N = 100,000')
grid minor

%% Plot density distribution of X
figure
subplot(2, 1, 1)
histogram(x, 50, 'FaceColor', 'b', 'EdgeColor', 'b')
xlim([-1, 3])
[f,xi] = ksdensity(x);
hold on
ylabel('Sample Distribution')
yyaxis right
plot(xi,f, 'r');
ylabel('PDF')
title('N(1, .5), N = 1000')
grid minor

subplot(2, 1, 2)
hold on
for i=1:1000
    plot([x(i) x(i)], [0, 1], 'b', 'LineWidth', .75)
    xlim([-1, 3])
end

%% Part B: Apply non-linear transformation and plot Y, Y with density function

figure
subplot(2, 1, 1)
histogram(x.^2, 100, 'FaceColor', 'b', 'EdgeColor', 'b')

xlim([-1, 3])
[f,xi] = ksdensity(x.^2);
hold on
ylabel('Sample Distribution')
yyaxis right
plot(xi,f, 'r');
ylabel('PDF')
title('Square of N(1, .5), N = 1000')
grid minor
xlabel('Y = X^2')

subplot(2, 1, 2)
hold on

for i=1:1000
    plot([x(i)^2 x(i)^2], [0, 1], 'b', 'LineWidth', .75)
    xlim([-1, 3])
end
%% Generate N samples from N(1, .5)
function [sample_list] = sample_n_from_normal_shift(variance, n, average)
    
    sample_list = [];
    for i=1:n
        sample_list = [sample_list sample_from_normal(variance, average)];
    end

    function [sample] = sample_from_normal(variance, average)
       all_rand = (2 * variance) .* rand(12, 1) - variance ;
       sample = average + sum(all_rand)/2;
    end

end