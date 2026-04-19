clear all;
close all;
clc;

% Definir parámetros del sistema
T = 10e-6; % Período de muestreo (s)
C = 1020e-6; % Capacitancia (F)
L = 1e-3; % Inductancia (H)
Vo = 800; % Voltaje de salida nominal (V)
P = 5000; % Potencia nominal (W)
R = Vo^2/P; %(ohm)
Vg = 230; %(Vrms)
Vp = Vg*sqrt(2); %(V)

% Ie igual para todas las fases
Ie = 2*Vo^2/(3*Vp*R); % Amplitud de la referencia


% Calcular el polo zp 
zp = 1 - (T/C) * ((3*Vp*Ie)/(2*Vo^2) + (1/R));


% Ganancias de las funciones de transferencia
gain_Hie = (3*T*Vp)/(2*C*Vo); % Ganancia para Hie
gain_Hvp = (3*T*Ie)/(2*C*Vo); % Ganancia para Hvp


% Definir las funciones de transferencia en el dominio discreto
% Numerador y denominador para Hie
num_Hie = gain_Hie; 
den_Hie = [1, -zp]; % Denominador: z - zp
Hie = tf(num_Hie, den_Hie, T); % Función de transferencia para ie

% Numerador y denominador para Hvp
num_Hvp = gain_Hvp;
den_Hvp = [1, -zp]; % Denominador: z - zp
Hvp = tf(num_Hvp, den_Hvp, T); % Función de transferencia para vp


t = 1; % Tiempo de simulación (1 ms)

% Respuesta al escalón para Hie
[y_ie, t_ie] = step(Hie, t);

% Respuesta al escalón para Hvp
[y_vp, t_vp] = step(Hvp, t);

% Graficar las respuestas
figure('Position', [100, 100, 1000, 600]);

% Respuesta de Hie
subplot(2, 1, 1);
plot(t_ie, y_ie, 'b', 'LineWidth', 2);
grid on;
title('Respuesta al Escalón de H_{ie}(z)');
xlabel('Tiempo (s)');
ylabel('Amplitud');

% Respuesta de Hvp
subplot(2, 1, 2);
plot(t_vp, y_vp, 'r', 'LineWidth', 2);
grid on;
title('Respuesta al Escalón de H_{v}(z)');
xlabel('Tiempo (s)');
ylabel('Amplitud');


% Título
sgtitle('Respuesta de las Funciones de Transferencia');

% Funciones de transferencia:
disp('Función de transferencia Hie:');
Hie
disp('Función de transferencia Hvp:');
Hvp


% Parámetros del controlador PID
Kp = 6.7016;
Ki = 2726.3288;
Kd = 0.00019818;
Ts = 10e-6; % Periodo de muestreo

% Cálculo de los coeficientes de la función de transferencia Gc(z)
% Numerador: b0*z^2 + b1*z + b2
b0 = Kd;
b1 = Kp*Ts - 2 * Kd;
b2 = Ki * Ts^2 + Kd - Kp*Ts;

% Denominador: a0*z + a1
a0 = Ts;
a1 = -Ts;

% Vectores de coeficientes
num_Gc = [b0, b1, b2];
den_Gc = [a0, a1]; 

% Crear el objeto de función de transferencia discreta
Gc = tf(num_Gc, den_Gc, Ts);

% Mostrar la función de transferencia resultante
disp('Función de transferencia Gc(z):');
Gc
%pidTuner(Hie)
%sisotool(Hie,Gc)