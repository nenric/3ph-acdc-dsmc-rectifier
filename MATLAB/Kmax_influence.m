clear all;
close all;
clc;

% Definir parámetros del sistema
T = 10e-6; % Período de muestreo (s)

% --- Definir la Planta Reformulada para el LGA (delay) ---
% G_planta = 1 / (z - 1)
num_G_planta = [1];            % Numerador (constante 1)
den_G_planta = [1, -1];        % Denominador (z - 1)

G_planta = tf(num_G_planta, den_G_planta, T);
%G_planta
%%sisotool(G_planta)
    

% --- Definir la Planta Reformulada para el LGA (delay) ---
% G_planta_de(z) = 1 / (z*(z-1)) = 1 / (z^2 - z)
num_G_planta_de = [1];             % Numerador (constante 1)
den_G_planta_de = [1, -1, 0];      % Denominador (z^2 - z + 0)

G_planta_de = tf(num_G_planta_de, den_G_planta_de, T);
G_planta_de
sisotool(G_planta_de)







