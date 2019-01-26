clear all;
close all;
clc;
%% Creating the model

%% Motor Speed
% A = [-10 1; -0.02 -2];
% B = [0; 2];
% C = [1 0];
% 
% Motor_speed_s = idss(A,B,C,0,'Ts',0);
% Continuos_model = Motor_speed_s;

%% Pitch and Plunge Model
A = [-0.313 56.7 0; -0.0139 -0.426 0; 0 56.7 0];
B = [0.232; 0.0203; 0.0];
C = [0 0 1];

PANDP = idss(A,B,C,0,'Ts',0);
Continuos_model = PANDP;

%% plot the impulse response for discrete and continuos
figure(1)
subplot(2,2,1)
impulse(Continuos_model);

%% Second Approach -- Phase Lag and Lead

% Initialize the poles and zeros
p_r1 = 2; p_i1 = 2;
z_r = 5; kc = 5;

% Set the optimization problem
maxITER = 40;
options = optimset('Display','iter','TolFun',1e-3,...
    'MaxIter',maxITER,'TolX',1e-3);

[x,fval,exitflag,output] = fminsearch(@QuadraticErrorPLL,...
    [p_r1, p_i1, z_r, kc],options,A,B,C);

pr1 = x(1); pi1 = x(2)*1i; 
zr = x(3); ks = x(4);

sim_opts = simset('DstWorkspace','current','SrcWorkspace','current');
sim('PitchANDPlunge.slx',100,sim_opts);

figure(4)
plot(y_ref)
hold on
plot(y_hat)

%% -----------------------------------------
function C = QuadraticErrorPLL(K,A,B,C) 
    Tsim = 500; %segundos
    
    pr1 = K(1); pi1 = K(2)*1i;
    zr = K(3); ks = K(4);
    
    sim_opts = simset('DstWorkspace','current','SrcWorkspace','current');
    sim('PitchANDPlunge.slx',Tsim,sim_opts);
    
    figure(1)
    subplot(2,2,2)
    plot(control_signal); %hold on;
    %plot(control_signal_filtered); hold off;
    subplot(2,2,3)
    plot(y_hat); hold on;
    plot(y_ref); hold off;
    
    C = (y_hat-y_ref)'*(y_hat-y_ref)/length(y_hat);
    
    figure(1)
    subplot(2,2,4)
    ax = gca;
    ax.XAxisLocation = 'origin';
    ax.YAxisLocation = 'origin';
    
    sz = (100 - C)/2;
    if (sz > 50)
        sz = 50;
    elseif (sz < 0)
        sz = 2;
    end
    
    %if K(1) < 100 && K(2) < 100
        scatter(-K(1),K(2),sz,'r','x')
        hold on
        scatter(-K(1),-K(2),sz,'r','x')
        hold on
    %end
    
    scatter(ks,0,sz,'g','*')
    hold on 
    scatter(-zr,0,sz,'k','o')
    
end
