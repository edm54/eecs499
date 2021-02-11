%{
Eric Miller
Edm54
EECS 499

P4) Mixed Gaussian

This file holds the code used to generate the plots for P4. 
Note that most of this code is for generating plots, as the mixed gaussian sampling
algorithm is in the function at the bottom. 

%}

%% Plot for N = 100,000
x = sample_n_from_mix(100000)
figure
hist(x, 1000)
xlim([-5, 5])
[f,xi] = ksdensity(x);
hold on
ylabel('Sample Distribution')
yyaxis right
plot(xi,f, 'g');
ylabel('PDF')
title('Mixed Distribution, N = 100,000')
grid minor

%% This function generates n samples from the mixed distribution
function [ sample_list] = sample_n_from_mix(n)

    sample_list = [];
    
    for i=1:n
        % Sample from .3 and .7 to get correct distribution 
        p = rand(1, 1) ;
        if p < .3
            sam = sample_from_normal(1);
        else
            sam = sample_from_normal2();
        end       
        %  sample_list = [sample_list sam1];
        sample_list = [sample_list sam];
    end
    
    function [sample] = sample_from_normal(variance)
       all_rand = (2 * variance) .* rand(12, 1) - variance ;
       sample = sum(all_rand)/2;
    end
    
    function [sample] = sample_from_normal2()
        variance = .5
       all_rand = (2 * variance) .* rand(12, 1) - variance ;
       sample = 2 + sum(all_rand)/2;
    end
    
end
