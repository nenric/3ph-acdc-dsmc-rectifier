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

% Respuesta de Hie
subplot(1, 1, 1);
plot(t_ie, y_ie, 'b', 'LineWidth', 2);
grid on;
title('Respuesta al Escalón de H_{ie}(z)');
xlabel('Tiempo (s)');
ylabel('Amplitud');


% Parámetros del controlador PID
Kp = 0.4585;
Ki = 25.4122;
Kd = 2.2919e-6;
Ts = 10e-6; % Periodo de muestreo

% Cálculo de los coeficientes de la función de transferencia Gc(z)
% Numerador: q0*z^2 + q1*z + q2
q0 = Kp + (Ts/2)*Ki + Kd/Ts;
q1 = -Kp + (Ts/2)*Ki -2*Kd/Ts;
q2 = Kd/Ts;

% Vectores de coeficientes
num_Gv = [q0, q1, q2];
den_Gv = [1, -1, 0]; 

% Crear el objeto de función de transferencia discreta
Gv = tf(num_Gv, den_Gv, Ts);

Gv


load PIDRstep.txt 
    
    % Extraer columnas en variables (ajusta los índices si tu archivo tiene un orden diferente)
    tiempo1    = PIDRstep(:, 1); % Columna 1 para el tiempo
    vo1       = PIDRstep(:, 2); % Columna 2 para d_a (azul, subplot 1)
    i_a1       = PIDRstep(:, 3); % Columna 5 para i_a (azul, subplot 2)
    i_b1       = PIDRstep(:, 4); % Columna 6 para i_b (rojo, subplot 2)
    i_c1       = PIDRstep(:, 5); % Columna 7 para i_c (amarillo, subplot 2)

load PIDVrefstep.txt 
    
    % Extraer columnas en variables (ajusta los índices si tu archivo tiene un orden diferente)
    tiempo2    = PIDVrefstep(:, 1); % Columna 1 para el tiempo
    vo2       = PIDVrefstep(:, 2); % Columna 2 para d_a (azul, subplot 1)
    i_a2       = PIDVrefstep(:, 3); % Columna 5 para i_a (azul, subplot 2)
    i_b2       = PIDVrefstep(:, 4); % Columna 6 para i_b (rojo, subplot 2)
    i_c2       = PIDVrefstep(:, 5); % Columna 7 para i_c (amarillo, subplot 2)
 
 % Configuración de fuentes
tamanoFuenteEtiquetas = 18;
tamanoNumerosEjes = 16;
tamanoTituloFigura = 12;
tamanoLeyenda = 18;
ax = gca;
set(ax, 'FontSize', tamanoNumerosEjes)

% --- Crear Gráficos 1 ---
figure;

% Subgráfico 1: vo
h_ax1 = subplot(2, 1, 1);
plot(tiempo1, vo1, 'b', 'LineWidth', 1.5); % Señal v_a en azul
ylabel_h1 = ylabel('$v_{o}$ (V)', 'Interpreter', 'latex');
set(h_ax1, 'XTickLabel', []); % Ocultar etiquetas de marcas del eje X
ylim([770, 830]); % Ajusta los límites del eje Y según tus datos
grid on;
set(ylabel_h1, 'FontSize', tamanoFuenteEtiquetas);

% Subgráfico 2: i_a, i_b, i_c
h_ax2 = subplot(2, 1, 2);
plot(tiempo1, i_a1, 'b', 'LineWidth', 1.5); % Señal i_a en azul
hold on;
plot(tiempo1, i_b1, 'r', 'LineWidth', 1.5); % Señal i_b en rojo
plot(tiempo1, i_c1, 'y', 'LineWidth', 1.5); % Señal i_c en amarillo
hold off;
ylim([-25, 25]); % Ajusta los límites del eje Y según tus datos (ej. max(abs(i_a))*1.1)
grid on;
ylabel_h2 = ylabel('$i_a, i_b, i_c$ (A)', 'Interpreter', 'latex');
xlabel_h2 = xlabel('Tiempo (s)','Interpreter', 'latex');
set(ylabel_h2, 'FontSize', tamanoFuenteEtiquetas);
set(xlabel_h2, 'FontSize', tamanoFuenteEtiquetas);

% --- Crear Gráficos 2 ---
figure;

% Subgráfico 1: vo
h_ax3 = subplot(2, 1, 1);
plot(tiempo2, vo2, 'b', 'LineWidth', 1.5); % Señal v_a en azul
ylabel_h3 = ylabel('$v_{o}$ (V)', 'Interpreter', 'latex');
set(h_ax3, 'XTickLabel', []); % Ocultar etiquetas de marcas del eje X
ylim([790, 820]); % Ajusta los límites del eje Y según tus datos
grid on;
set(ylabel_h3, 'FontSize', tamanoFuenteEtiquetas);

% Subgráfico 2: i_a, i_b, i_c
h_ax4 = subplot(2, 1, 2);
plot(tiempo2, i_a2, 'b', 'LineWidth', 1.5); % Señal i_a en azul
hold on;
plot(tiempo2, i_b2, 'r', 'LineWidth', 1.5); % Señal i_b en rojo
plot(tiempo2, i_c2, 'y', 'LineWidth', 1.5); % Señal i_c en amarillo
hold off;
ylim([-20, 20]); % Ajusta los límites del eje Y según tus datos (ej. max(abs(i_a))*1.1)
grid on;
ylabel_h4 = ylabel('$i_a, i_b, i_c$ (A)', 'Interpreter', 'latex');
xlabel_h4 = xlabel('Tiempo (s)','Interpreter', 'latex');
set(ylabel_h4, 'FontSize', tamanoFuenteEtiquetas);
set(xlabel_h4, 'FontSize', tamanoFuenteEtiquetas);



