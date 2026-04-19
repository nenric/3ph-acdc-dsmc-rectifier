clear all;
close all;
clc;

% Definir parámetros del sistema
T = 50e-6; % Período de muestreo (s)
C = 1020e-6; % Capacitancia (F)
L = 1e-3; % Inductancia (H)
V0 = 130; % Voltaje de salida nominal (V)
P = 5000; % Potencia nominal (W)
R=3.38; %(ohm)

% Gx y Vgx son iguales para todas las fases
Gx = 3.704; % Valor nominal de Gx (igual para todas las fases)
Vgx = 30; % Valor nominal de Vgx (igual para todas las fases)

% Calcular el término de la suma para las tres fases (a, b, c)
sum_Gx_Vgx2 = 3 * Gx * Vgx^2;

% Calcular el polo zp 
zp = 1 + (T / (C * V0^2)) * (1/R - 2 * sum_Gx_Vgx2);

% Calcular los ceros zc,gx y zc,vgx
zc_gx = 1 + T / (L * Gx);
zc_vgx = 1 + 2 * T / (L * Gx);

% Ganancias de las funciones de transferencia
gain_Hgx = -2 * L * Gx * Vgx^2 / (C * V0); % Ganancia para Hgx
gain_Hvgx = -2 * L * Gx^2 * Vgx / (C * V0); % Ganancia para Hvgx
gain_Hp = -T / (C * V0); % Ganancia para Hp

% Definir las funciones de transferencia en el dominio discreto
% Numerador y denominador para Hgx
num_Hgx = gain_Hgx * [1, -zc_gx]; % Numerador: gain * (z - zc,gx)
den_Hgx = [1, -zp]; % Denominador: z - zp
Hgx = tf(num_Hgx, den_Hgx, T); % Función de transferencia para g_x

% Numerador y denominador para Hvgx
num_Hvgx = gain_Hvgx * [1, -zc_vgx]; % Numerador: gain * (z - zc,vgx)
den_Hvgx = [1, -zp]; % Denominador: z - zp
Hvgx = tf(num_Hvgx, den_Hvgx, T); % Función de transferencia para v_gx

% Numerador y denominador para Hp
%num_Hp = gain_Hp * [1]; % Numerador: gain
%den_Hp = [1, -zp]; % Denominador: z - zp
%Hp = tf(num_Hp, den_Hp, T); % Función de transferencia para p


t = 1e-3; % Tiempo de simulación (1 ms)

% Respuesta al escalón para Hgx
[y_gx, t_gx] = step(Hgx, t);

% Respuesta al escalón para Hvgx
[y_vgx, t_vgx] = step(Hvgx, t);

% Respuesta al escalón para Hp
%[y_p, t_p] = step(Hp, t);

% Graficar las respuestas
figure('Position', [100, 100, 1000, 600]);

% Respuesta de Hgx
subplot(3, 1, 1);
plot(t_gx, y_gx, 'b', 'LineWidth', 2);
grid on;
title('Respuesta al Escalón de H_{g_x}(z) (para una fase x)');
xlabel('Tiempo (ms)');
ylabel('Amplitud');

% Respuesta de Hvgx
subplot(3, 1, 2);
plot(t_vgx, y_vgx, 'r', 'LineWidth', 2);
grid on;
title('Respuesta al Escalón de H_{v_{gx}}(z) (para una fase x)');
xlabel('Tiempo (ms)');
ylabel('Amplitud');

% % Respuesta de Hp
% subplot(3, 1, 3);
% plot(t_p, y_p, 'g', 'LineWidth', 2);
% grid on;
% title('Respuesta al Escalón de H_p(z)');
% xlabel('Tiempo (ms)');
% ylabel('Amplitud');

% Título
sgtitle('Respuesta de las Funciones de Transferencia');

% Funciones de transferencia:
disp('Función de transferencia Hgx:');
Hgx
disp('Función de transferencia Hvgx:');
Hvgx
% disp('Función de transferencia Hp:');
% Hp