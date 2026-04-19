clc, clear, close all;

%% Parámetros de simulación (comunes para ambas gráficas)
Vp = 1;
fs = 20e3;          % Frecuencia de muestreo y conmutación
Ts = 1/fs;          % Tiempo de muestreo
fgrid = 50;         % Frecuencia de la red
w = 2*pi*fgrid;     % Frecuencia angular de la red
N_cycle = 3;        % Número de ciclos
t_end = N_cycle/fgrid; % Tiempo de simulación

% Señales de potencia
t = linspace(0, t_end, round(t_end/Ts));
vg = Vp*sin(w*t);

%% Creación de la figura y los subplots
combined_figure = figure;
combined_figure.Name = 'Aproximaciones de la Tensión de Red'; % Nombre de la ventana
combined_figure.Visible = 'on';

% -------------------------------------------------------------------------
% SUBPLOT 1: Solución matemática (retraso de 1 Ts)
% -------------------------------------------------------------------------
subplot(2, 1, 1); % Crea una cuadrícula 2x1 y selecciona la primera posición

% Cálculo de la aproximación
theta = w*Ts;       % Parámetro del modelo
vg_1 = zeros(1, length(t));
vg_1(1) = 0;
for i=2:length(t)-1
    vg_1(i+1) = vg(i)*cos(theta) + sqrt(abs(1 - vg(i)^2))*sin(theta);
end

% Gráfica
plot(t, vg, 'Color', 'b', 'LineWidth', 1.5, 'DisplayName', "Referencia sinusoidal $s(n)$");
hold on;
plot(t, vg_1, 'Color', 'r', 'LineWidth', 1.5, 'LineStyle', '--', 'DisplayName', "Aproximaci\'on $\hat{s}(n)$");
grid minor;
hold off;

% Formato y etiquetas
ax1 = gca;
ax1.FontSize = 16;
ax1.TickLabelInterpreter = "latex";
ax1.YLim = [-1.2*Vp, 1.2*Vp];
set(ax1, 'XTickLabel', []);
ylabel('$s(t)$', 'interpreter', 'latex', 'FontSize', 20);
lgd1 = legend;
lgd1.Interpreter = "latex";
lgd1.FontSize = 18;

% -------------------------------------------------------------------------
% SUBPLOT 2: Solución recurrente (retraso de 2 Ts)
% -------------------------------------------------------------------------
subplot(2, 1, 2); % Selecciona la segunda posición de la cuadrícula 2x1

% Cálculo de la aproximación
k = 2*cos(w*Ts) + 1; % Parámetro del modelo
vg_2 = zeros(1, length(t));
mem = [0, 0];       % mem = [m(n-1), m(n-2)]
for i=1:length(t)-1
    vg_2(i+1) = k*(vg(i) - mem(1)) + mem(2);
    % Actualización de memoria
    mem(2) = mem(1);
    mem(1) = vg(i);
end

% Gráfica
plot(t, vg, 'Color', 'b', 'LineWidth', 1.5, 'DisplayName', "Referencia sinusoidal $s(n)$");
hold on;
plot(t, vg_2, 'Color', 'r', 'LineWidth', 1.5, 'LineStyle', '--', 'DisplayName', "Aproximaci\'on $\hat{s}(n)$");
grid minor;
hold off;

% Formato y etiquetas
ax2 = gca;
ax2.FontSize = 16;
ax2.TickLabelInterpreter = "latex";
ax2.YLim = [-1.2*Vp, 1.2*Vp];
xlabel('Tiempo (s)', 'interpreter', 'latex', 'FontSize', 20);
ylabel('$s(t)$', 'interpreter', 'latex', 'FontSize', 20);
lgd2 = legend;
lgd2.Interpreter = "latex";
lgd2.FontSize = 18;