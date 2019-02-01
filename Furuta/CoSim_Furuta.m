clf;
clf;
clc;

Tsim = 50;
%% Furta Model

%%Linear
A = [0    1.0000         0         0;
     0  -18.3375  -53.8799    0.2540;
     0         0         0    1.0000;
     0    9.4929  104.8377   -0.4942];
     
B = [0; 35.2932; 0; -18.2704];

C = [ 0     0     1     0;
      0     0     0     1];
 
PANDP = idss(A,B,C,0,'Ts',0);
Continuos_model = PANDP;


%%Non Linear
g = 9.81;

m1 = 0.380;
m2 = 0.054;
L1 = 0.066; 
L2 = 0.146;  
M = 0.044;

J = 3.5256e-04;            %System
kb_p = 4.7940e-04 ;
kb_m = 6.75e-4 ;
b = 2.5482e-4;

ke = 0.5; % kt Motor 2e-2
Re = 14.5;  %Medido

alpha = J + (M+m1/3+m2)*L1^2;
beta = (M + m2/3)*L2^2;
gamma = (M + m2/2)*L2*L1;
sigma = (M + m2/2)*g*L2;

initial_state = pi;
Ts = 0.01;
dtDisc = 0.01;
Reference = [0 0 0 0];
Zn = 3;                 %Dead Zone
GSwingUp = 3;

%% Feedback control
% Initialize K
K = [ -0.2511   -1.2029  -16.3580   -2.2683];
%[ -0.4472   -1.3297  -36.7079   -3.9317];


% Set the optimization problem
maxITER = 100;
options = optimset('Display','iter','PlotFcns',@optimplotfval,'TolFun',1e-3,...
    'MaxIter',maxITER,'TolX',1e-3);

[x,fval,exitflag,output] = fminsearch(@QuadraticErrorPLL,...
    K,options,...
   Tsim, J, kb_p, kb_m, b, ke, Re, alpha, beta, gamma, sigma, g, initial_state, Ts, dtDisc, GSwingUp);

K = x;


sim_opts = simset('DstWorkspace','current','SrcWorkspace','current');
sim('Furuta_Control_all_states.slx',100,sim_opts);

figure(4)
subplot(1,2,1)
hold on
plot(y_ref(:,1))
plot(y_hat(:,1))
subplot(2,1,2)
plot(y_hat(:,3))
%% -----------------------------------------
function C = QuadraticErrorPLL(K,Tsim, J, kb_p, kb_m, b, ke, Re, alpha, beta, gamma, sigma, g, initial_state, Ts, dtDisc,GSwingUp) 
    
    Q = [10 0 0   0;
         0 2 0   0;
         0 0 10  0;
         0 0 0   1];
    R = 10;
       
    sim_opts = simset('DstWorkspace','current','SrcWorkspace','current');
    sim('Furuta_Control_all_states.slx',Tsim,sim_opts);
         
    y_hat_norm = (y_hat - min(y_hat))./(max(y_hat)-min(y_hat));
    
    y_ref_norm = (y_ref - min(y_hat))./(max(y_hat)-min(y_hat));
     
    control_signal_norm = (control_signal - min(control_signal))./(max(control_signal) - min(control_signal));
    C = (y_hat_norm-y_ref_norm)'*(y_hat_norm-y_ref_norm)/length(y_hat_norm);
     
    Control = (control_signal)'*(control_signal)/length(control_signal);
     
    C = sum(diag(C*Q))+Control*R;
    
    
    figure(1)    

    subplot(3,1,1)    
    plot(y_hat(:,1)); hold on;
    plot(y_ref(:,1)); hold off;
    legend('Real','Setpoint');
    title('Phi');
    
    subplot(3,1,2)    
    
    plot(y_hat(:,3)); hold on;
    plot(y_ref(:,3)); hold off;
    legend('Real','Setpoint');
    title('Theta');
    
      
    subplot(3,1,3)
    axis([-30 0 20 50]); 

    
    ax = gca;
    ax.XAxisLocation = 'origin';
    ax.YAxisLocation = 'origin';
    
    sz = 20
      
    abs(sum(K))
    
    if ( abs(sum(K)) < 100)
     scatter(K(1),C,sz,[0, 0, 1],'x')
     hold on
     scatter(K(2),C,sz,[1, 0, 0],'x')
     hold on
     scatter(K(3),C,sz,[0, 0.5, 0],'x')
     hold on
     scatter(K(4),C,sz,[0.75, 0, 0.75],'x')
     hold on
    end
     legend('K1','K2','K3','K4');
     title('Cost x K');
     
    
end
