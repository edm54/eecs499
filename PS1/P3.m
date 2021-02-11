%{
Eric Miller
Edm54
EECS 499

P3) Rejection Sampling

This file holds the code used to generate the plots for P3. 
Note that most of this code is for generating plots, as the rejection sampling
algorithm is in the function at the bottom. 

%}

%% Plot 100,000 samples
x = sample_n_from_abs(100000);
figure
hist(x, 1000)
xlim([-2, 2])
[f,xi] = ksdensity(x);
hold on
ylabel('Sample Distribution')
yyaxis right
plot(xi,f);
ylabel('PDF')
title('Rejection Sampling, N = 100,000')
grid minor

%% Generate samples from F(x) via rejection sampling
function [sample_list] = sample_n_from_abs(n)
    sample_list = []
    for i=1:n
        sample_list = [sample_list sample_from_abs()];
    end
    function [sample] = sample_from_abs()
       f = 100;
       y = 1000;
       while y > f  
            x = 4 * rand(1, 1) - 2;
            f = abs(x);
            if f>1
                f = 0;
            end
            
            y = rand(1, 1) * 2  ;  
       end
       sample = x;
    end    
end