% setup
a = 5; b = 8; c = 3;

NCS.A = [a-b 0.5-c;
         0   1];

NCS.B = [0; 1];

NCS.nx = 2;

syms h s k1 k2 k3 k4 k5

NCS.h = h;

a1 = 1.0;
b1 = 1.0;
NCS.poles = [-a1+b1*1i -a1-b1*1i];

NCS.K = place(NCS.A, NCS.B, NCS.poles);
eig(NCS.A - NCS.B * NCS.K);

NCS.K = [k1 k2 k3 k4 k5];

clear ans a1 b1
%% Q4.1
% small delay with tau = 0.5h
small_tau = 0.5*h;

NCS_small_delay.Fx = expm(NCS.A * NCS.h);

temp = expm(NCS.A * s);

NCS_small_delay.Fu = int(temp, s, h-small_tau, h) * NCS.B;
NCS_small_delay.G1 = int(temp, s, 0, h-small_tau) * NCS.B;

clear temp
% augmentation
NCS_small_delay.F = [NCS_small_delay.Fx NCS_small_delay.Fu zeros(2, 1) zeros(2, 1);
                     zeros(1, 2) 0 0 0;
                     zeros(1, 2) 1 0 0;
                     zeros(1, 2) 0 1 0];
NCS_small_delay.G = [NCS_small_delay.G1; 1; 0; 0];
% NCS_small_delay.F
% NCS_small_delay.G

% mid delay with tau = h
mid_tau = h;

NCS_mid_delay.Fx = expm(NCS.A * NCS.h);

temp = expm(NCS.A * s);

NCS_mid_delay.Fu1 = int(temp, s, h-mid_tau, h) * NCS.B;
NCS_mid_delay.Fu2 = int(temp, s, 0, h-mid_tau) * NCS.B;

clear temp
% augmentation
NCS_mid_delay.F = [NCS_mid_delay.Fx NCS_mid_delay.Fu1 NCS_mid_delay.Fu2 zeros(2, 1);
                   0, 0, 0, 0, 0;
                   0, 0, 1, 0, 0;
                   0, 0, 0, 1, 0];

NCS_mid_delay.G = [0; 0; 1; 0; 0];


% large delay with tau = h
large_tau = 2*h;

NCS_large_delay.Fx = expm(NCS.A * NCS.h);

temp = expm(NCS.A * s);

NCS_large_delay.Fu2 = int(temp, s, large_tau-2*NCS.h, large_tau) * NCS.B;
NCS_large_delay.Fu3 = int(temp, s, 0, large_tau-2*NCS.h) * NCS.B;

clear temp
% augmentation
NCS_large_delay.F = [NCS_large_delay.Fx, zeros(2,1), NCS_large_delay.Fu2, NCS_large_delay.Fu3;
                   0, 0, 0, 0, 0;
                   0, 0, 1, 0, 0;
                   0, 0, 0, 1, 0];
NCS_large_delay.G = [0; 0; 1; 0; 0];

% NCS_large_delay.F
% NCS_large_delay.G

NCS_small_delay.A = NCS_small_delay.F - NCS_small_delay.G * NCS.K;

NCS_mid_delay.A = NCS_mid_delay.F - NCS_mid_delay.G * NCS.K;

NCS_large_delay.A = NCS_large_delay.F - NCS_large_delay.G * NCS.K
A = NCS_small_delay.A*NCS_mid_delay.A*NCS_large_delay.A;

A_eig = vpa(subs(A, [h, k1, k2, k3, k4, k5], [0.19, -2, 0, 0, 0, 0]));

eig(A_eig)

max(double(abs(eig(A_eig))))
% A_eig = eig(NCS_small_delay.A*NCS_mid_delay.A*NCS_large_delay.A)
% 
% temp = vpa(subs(A_eig, [h, k1, k2, k3, k4, k5], [0.09, -2, 0, 0, 0, 0]));
% 
% max(double(abs(temp)))

obj0 = objfunx(-1, 0.004, 0.11, 0, 0, A, h, k1, k2, k3, k4, k5)

K0 = [-1, 0, 0.1, 0, 0];
T0 = 1e5; 
terminated = 1e-3;
rate = 0.95;

T = T0;
K = K0;
while(T > terminated)
    
    K_temp = deivation(K);
    
    temp = objfunx(K_temp(1), K_temp(2), K_temp(3), K_temp(4), K_temp(5), A, h, k1, k2, k3, k4, k5)
    
    if(temp > obj0)
        K = K_temp;
        obj0 = temp;
    else
        if(rand(1) > exp(-(temp - obj0)/T))
            K = K_temp;
            obj0 = temp;
        end
    end
    obj0
    K
    T = T*rate;
end

function K_new = deivation(K)
    K_new = zeros(1, length(K));
    for i = 1:length(K)
        r = -1 + (2)*rand(1);
        K_new(i) = K(i) + 0.02*r;
    end
end


function h1 = objfunx(K1, K2, K3, K4, K5, A, h, k1, k2, k3, k4, k5)
    h1 = 0;
    step = 0.01;
    while true
        h1 = h1 + step;
        A_eig = vpa(subs(A, [h, k1, k2, k3, k4, k5], [h1, K1, K2, K3, K4, K5]));
        spectral_radius = max(double(abs(eig(A_eig))));
        
        if (spectral_radius >= 1)
            h1 = h1 - step;
            break;
        end
    end
end
