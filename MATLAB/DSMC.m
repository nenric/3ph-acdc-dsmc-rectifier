close all;

% --- Cargar Datos ---
% Reemplaza 'tus_datos.csv' con el nombre de tu archivo.
load DSMCdatos2.txt 
    
    % Extraer columnas en variables (ajusta los índices si tu archivo tiene un orden diferente)
    tiempo      = DSMCdatos2(:, 1); % Columna 1 para el tiempo
    i           = DSMCdatos2(:, 2); % Columna 2 para d_a (azul, subplot 1)
    i_ref       = DSMCdatos2(:, 3); % Columna 3 para d_b (rojo, subplot 1)
    

% --- Crear Gráficos ---
figure;

% Subgráfico 1: v_a, v_b, v_c
h_ax1 = subplot(1, 1, 1);
plot(tiempo, i, 'b', 'LineWidth', 1.5); % Señal v_a en azul
hold on;
plot(tiempo, i_ref, 'r', 'LineWidth', 2); % Señal v_b en rojo
hold off;
grid on;
set(h_ax1, 'FontSize', 12);
ylabel('$i, i_{ref}$ (A)', 'Interpreter', 'latex', 'FontSize', 26);
xlabel(h_ax1, 'Tiempo (s)','Interpreter', 'latex', 'FontSize', 20); % Etiqueta del eje X solo en el último subgráfico
%xlim([39.4e-3, 40.6e-3]); % Ajusta los límites del eje Y según tus datos
xlim([0, 100.0e-3]);
legend('$i$','$i_{ref}$','Interpreter','latex','FontSize',20);

% Título general para la figura (opcional)
%sgtitle('Resultados de la Simulación');



% Ajustes adicionales (opcional):
% - Puedes ajustar los grosores de línea ('LineWidth').
% - Cambiar colores si es necesario (e.g., un amarillo más oscuro: [0.9290 0.6940 0.1250]).
% - Ajustar el tamaño de fuente de las etiquetas:
%   ylabel(h_ax1, '$d_a, d_b, d_c$', 'Interpreter', 'latex', 'FontSize', 12);
%   xlabel(h_ax4, 'Time (s)', 'FontSize', 12);