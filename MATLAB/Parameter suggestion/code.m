clc 
close all
% Read data from second row (header is text)
data = readtable("datosRespuestaFinal.xlsx");
t = data{2:end,1};
u = data{2:end,2};
y = data{2:end,3};
% Convert time from microseconds to seconds and start from 0
t = t - t(1);
t = t * (10^-6);
% Plot orignal response
plot(t, y, ".-");
xlim([0 3])
title("Step response")

% Delete elements before step entrance
t = t(21:end);
t = t - t(1);
u = u(21:end);
y = y(21:end);
%% Get system
% Get the mean of values after stabilization
valStable = mean(y(237:end));
% Get value of gain of the system (stable value / the input (255))
K = valStable / u(1);
% Find  vuelues of t1 and t2
val63 = valStable*(1-exp(-1));
[~, i63] = min(abs(val63-y));
t63 = t(i63);
val28 = valStable*(1-exp(-1/3));
[~, i28] = min(abs(val28-y));
t28 = t(i28);

% Find model 
Tao = 3/2 * (t63-t28);
Teta = t63 - Tao;
G = tf(K, [Tao, 1]);


figure
plot(t, y, "r.", MarkerSize=10)
hold on
tin = t(1):0.01:t(end); 
opt = stepDataOptions('StepAmplitude',u(1));
[yOut,tOut] = step(G, tin, opt);
plot(tOut, yOut, 'b', LineWidth=2);
xlim([0 0.45])
title("Step response")
hold off

%% Compera model to filttered data
y_fil = smooth(y, 10);
figure
scatter(t, y_fil)
hold on
opt = stepDataOptions('StepAmplitude',u(1));
step(G, opt);
hold off
%% Find control system parameters

% Find recommended sampling time
val95 = valStable*0.95;
[~, i95] = min(abs(val28-y));
t95 = t(i95);
sampling_time = (t95/12 + t95/10) / 2;
fprintf("Recommended sampling time %f sec.\n", sampling_time);

% Get PID paramters
pidtune(G, "PID")

%% Coeficientes de controladores P, PI y PID con Primer método de Ziegler-Nichols
% Primer método de Ziegler-Nichols
C_ZN1_P = Tao/(K*Teta)
C_ZN1_PI = [0.9*Tao/(K*Teta) 3.3*Teta]
C_ZN1_PID = [1.2*Tao/(K*Teta) 2*Teta 0.5*Teta]

%% Coeficientes de controladores PID con criterios integrales
% PID ISE
C_ISE_KC = 1.474/K*(Teta/Tao)^-0.97;
C_ISE_TI = Tao/1.115*(Teta/Tao)^-0.753;
C_ISE_TD = 0.550*Tao*(Teta/Tao)^0.948;
CPID_ISE = [C_ISE_KC C_ISE_TI C_ISE_TD];

% PID IAE cambio de referencia
C_IAE_REFE_KC = 1.086/K*(Teta/Tao)^-0.869;
C_IAE_REFE_TI = Tao/(0.74-0.130*(Teta/Tao));
C_IAE_REFE_TD = 0.348*(Tao)*(Teta/Tao)^0.914;
CPID_IAE_REFE = [C_IAE_REFE_KC C_IAE_REFE_TI C_IAE_REFE_TD];

% PID IAE Rechazo de perturbaciones
C_IAE_RECHA_KC = 1.435/K*(Teta/Tao)^-0.921;
C_IAE_RECHA_TI = Tao/0.878*(Teta/Tao)^0.749;
C_IAE_RECHA_TD = 0.482*Tao*(Teta/Tao)^1.137;
CPID_IAE_RECHA = [C_IAE_RECHA_KC C_IAE_RECHA_TI C_IAE_RECHA_TD];

% PID ITAE cambio de referencia
C_ITAE_REFE_KC = 0.965/K*(Teta/Tao)^-0.855;
C_ITAE_REFE_TI = Tao/(0.796-0.147*(Teta/Tao));
C_ITAE_REFE_TD = 0.308*Tao*(Teta/Tao)^0.992;
CPID_ITAE_REFE = [C_ITAE_REFE_KC C_ITAE_REFE_TI C_IAE_REFE_TD];

% PID ITAE Rechazo de perturbaciones
C_ITAE_RECHA_KC = 1.357/K*(Teta/Tao)^-0.947;
C_ITAE_RECHA_TI = Tao/0.842*(Teta/Tao)^0.738;
C_ITAE_RECHA_TD = 0.381*Tao*(Teta/Tao)^0.995;
CPID_ITAE_RECHA = [C_ITAE_RECHA_KC C_ITAE_RECHA_TI C_ITAE_RECHA_TD];
%% Coeficientes de controladores PI con criterios integrales

% PID IAE cambio de referencia
C_IAE_REFE_KP = 0.7586/K*(Teta/Tao)^-0.869;
C_IAE_REFE_KI = C_IAE_REFE_KP / (Tao/(1.02-0.323*(Teta/Tao)));
CPI_IAE_REFE = [C_IAE_REFE_KP C_IAE_REFE_KI]
    
% PID IAE Rechazo de perturbaciones
C_IAE_RECHA_KP = 0.984/K*(Teta/Tao)^-0.986;
C_IAE_RECHA_KI = C_IAE_RECHA_KP/ (Tao/0.608*(Teta/Tao)^0.707);
CPI_IAE_RECHA = [C_IAE_RECHA_KP C_IAE_RECHA_KI]

% PID ITAE cambio de referencia
C_ITAE_REFE_KP = 0.586/K*(Teta/Tao)^-0.916;
C_ITAE_REFE_KI = C_ITAE_REFE_KP/ (Tao/(1.03-0.165*(Teta/Tao)));
CPI_ITAE_REFE = [C_ITAE_REFE_KP C_ITAE_REFE_KI]

% PID ITAE Rechazo de perturbaciones
C_ITAE_RECHA_KP = 0.859/K*(Teta/Tao)^-0.977;
C_ITAE_RECHA_KI = C_ITAE_RECHA_KP/ (Tao/0.674*(Teta/Tao)^0.680);
CPI_ITAE_RECHA = [C_ITAE_RECHA_KP C_ITAE_RECHA_KI]

