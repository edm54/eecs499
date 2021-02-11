x = -1:.01:1
y = pdf('Normal', x, 0, .1) 
figure
plot(x, y)
samps = []
for i = 1:1000
    s = sample_from_normal(.01);
    samps = [samps s];
    
end

function [sample] = sample_from_normal(variance)
       all_rand = (2 * variance) .* rand(12, 1) - variance ;
       sample = sum(all_rand)/2;
end


