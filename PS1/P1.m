%{
Eric Miller
Edm54
EECS 499

P1) Sample From Normal

This file holds the code used to generate the plots for P1. 
Note that most of this code is for generating plots, as the normal sampling
algorithm is in the function at the bottom. 

%}

%% Gather Samples with various N
samples_100 = sample_n_from_normal(1, 100);
samples_1000 = sample_n_from_normal(1, 1000);
samples_100000 = sample_n_from_normal(1, 100000);

%% Plot N = 100,000
[f,xi] = ksdensity(samples_100000);
figure
hold on
histogram(samples_100000, 50)
title('Guassian Distribution, N(0, 1), N = 100,000')
yyaxis right
plot(xi,f);
ylabel('PDF')
grid minor
xlim([-4, 4])
xlabel('X')
%% Plot N = 1,000
samples_1000 = sample_n_from_normal(1, 1000);
[f,xi] = ksdensity(samples_1000);
figure
histogram(samples_1000, 50)
title('Guassian Distribution, N(0, 1), N = 1000')
yyaxis right
plot(xi,f);
ylabel('PDF')
grid minor
xlim([-4, 4])
xlabel('X')

%% Plot N = 100
[f,xi] = ksdensity(samples_100);
figure
hold on
ylabel('Sample Distribution')
histogram(samples_100, 15)
yyaxis right
plot(xi,f);
title('Guassian Distribution, N(0, 1), N = 100')
ylabel('PDF')
grid minor
xlabel('X')

%% This function generates N samples with variance from Normal Distribution
function [sample_list] = sample_n_from_normal(variance, n)
    
    sample_list = [];
    for i=1:n
        sample_list = [sample_list sample_from_normal(variance)];
    end

    function [sample] = sample_from_normal(variance)
       all_rand = (2 * variance) .* rand(12, 1) - variance ;
       sample = sum(all_rand)/2;
    end

end






