%% Decay functions go here
function G = decayFunc(r)
R = 0.5;
G = -(1/R).*r + 1;
nullme = find(G < 0);
G(nullme) = zeros(1,length(nullme));
end