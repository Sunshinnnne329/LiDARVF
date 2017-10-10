clc;
clear all;
close all;

funcList(1).name = 'Inverse Exponential';
funcList(1).func = @VInvExp;

funcList(2).name = 'Hyperbolic Tangent';
funcList(2).func = @VTanh;

funcList(3).name = 'Linear';
funcList(3).func = @VLin;

funcList(4).name = 'Gaussian';
funcList(4).func = @VGauss;

funcList(5).name = 'Inverse Sqaure';
funcList(5).func = @VInvSq;

save funcList

function G = VInvExp(rrin)
    a = 4;
    G = 1./exp(a.*rrin);
end

function G = VTanh(rrin)
    a = 1;
    rrt = -((rrin)*2*pi-pi);
    G = (tanh(a.*rrt)+1)/2;
end

%{
function G = VLogistic(rrin)
    rrt = -((rrin)*2*pi-pi);
    a = 1;
    G = 1./(1+exp(-a.*rrt));
end
%}

function G = VLin(rrin)
    R = 0.5;
    G = -(1/R).*rrin + 1;
    G(G < 0) = 0;
end

function G = VGauss(rrin)
    R = 0.5;
    sig = R./sqrt(2.*log(2));
    G = exp(-(rrin.^2)./(2.*sig.^2));
end

function G = VInvSq(rrin)
    a = 0.5;
    G = 1./(a.*rrin.^2);
end

