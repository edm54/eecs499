%{
Eric Miller
Edm54
EECS 499

P2) Uniform Sampling

This file holds the code used to generate the plots for P2. 
Note that most of this code is for generating plots, as the uniform sampling
algorithm is in the function at the bottom. 

%}


%% Generate Uniform Samples

samples_100 = sample_n_from_uniform(1.5, 100);
samples_1000 = sample_n_from_uniform(1.5, 1000);
samples_100000 = sample_n_from_uniform(1.5, 100000);


%% Plot N = 100,000

[f,xi] = ksdensity(samples_100000);
figure
hold on
histogram(samples_100000, 100)
title('Uniform Distribution, U(0, 1.5), N = 100,000')
yyaxis right
plot(xi,f);
ylabel('PDF')
grid minor
xlim([0, 1.5])


%% Plot N = 1000
samples_1000 = sample_n_from_uniform(1.5, 1000);
[f,xi] = ksdensity(samples_1000);
figure
histogram(samples_1000, 25)
title('Uniform Distribution, U(0, 1.5), N = 1000')
yyaxis right
plot(xi,f);
ylabel('PDF')
grid minor
xlim([0, 1.5])

%% Plot N = 100
samples_100 = sample_n_from_uniform(1.5, 100);
[f,xi] = ksdensity(samples_100);
figure
hold on
ylabel('Sample Distribution')
histogram(samples_100, 20)
yyaxis right
plot(xi,f);
title('Uniform Distribution, U(0, 1.5), N = 100')
ylabel('PDF')
grid minor
xlim([0, 1.5])


%% Generate n uniform samples in range(0, upper_lim)
function [sample_list] = sample_n_from_uniform(upper_lim, n)
    
    sample_list = [];
    for i=1:n
        sample_list = [sample_list sample_from_uniform(upper_lim)];
    end

    function [sample] = sample_from_uniform(upper_lim)
       sample =  upper_lim * rand(1, 1)  ;
    end

end


