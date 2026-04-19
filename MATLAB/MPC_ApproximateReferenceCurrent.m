clc, clear, close all;
%% Model Predictive control (MPC) for pre-regulator PCF 3-ph rectifier
% Loss-free resistor approach for a ac-dc rectifier using sliding mode
% controllers

% Parameters of simulation
Vp = 1;
fs = 20e3; % Switching frequency and sampling frequency
Ts = 1/fs; % Sampling time
fgrid = 50; % Grid frequency
w = 2*pi*fgrid; % Grid angular frequency
N_cycle = 3; % Number of cycles of the grid
t_end = N_cycle/fgrid; % Simulation time

% Power signals
t = linspace(0,t_end,round(t_end/Ts));
vg = Vp*sin(2*pi*60*t);

%% Approximate input voltage grid - MPC design

% Opc 1: mathematical solution
% One Ts delay
theta = w*Ts; % Parameter of model
vg_1 = zeros(1,length(t)); % vector of estimated vg value
vg_1(1) = 0; % Initial value
for i=2:length(t)-1
	% i corresponds to actual mensure value of vg
	% The sqrt needs a abs() for real values
	vg_1(i+1) = vg(i)*cos(theta)+sqrt(abs(1-vg(i)^2))*sin(theta);
end

signals1 = figure;
signals1.Visible = 'on';
plot(t,vg,'Color','b','LineWidth',1.5,'DisplayName',"Referencia sinusoidal $s(n)$"); % Input voltage
    grid minor, hold on;
plot(t,vg_1,'Color','r','LineWidth',1.5,'LineStyle','--','DisplayName',"Aproximacion $\hat{s}(n)$")
ax = gca; ax.FontSize = 16; ax.TickLabelInterpreter = "latex";
ax.YLim = [-1.2*Vp,1.2*Vp];
title(signals1.Name,"FontSize",14,'Interpreter','latex');
xlabel('Tiempo (s)','interpreter','latex','FontSize',20);
ylabel('$s(t)$','interpreter','latex','FontSize',20);
lgd = legend; lgd.Interpreter = "latex";
set(lgd, 'FontSize', 18);

% Opc 2: recurrent solution
% Two Ts delay. Requires storing 2 data of input voltage in memory
k = 2*cos(w*Ts)+1; % Parameter of model
vg_2 = zeros(1,length(t)); % vector of estimated vg value
mem = [0,0]; %initial values mem = [m(n-1),m(n-2)]
for i=1:length(t)-1
	% i corresponds to actual mensure value of vg
	vg_2(i+1) = k*(vg(i)-mem(1)) + mem(2);

	% Storing in memory
	mem(2) = mem(1); mem(1) = vg(i);
end

signals2 = figure;
signals2.Visible = 'on';
plot(t,vg,'Color','b','LineWidth',1.5,'DisplayName',"Referencia sinusoidal $s(n)$"); % Input voltage
    grid minor, hold on;
plot(t,vg_2,'Color','r','LineWidth',1.5,'LineStyle','--','DisplayName',"Aproximacion $\hat{s}(n)$")
ax = gca; ax.FontSize = 16; ax.TickLabelInterpreter = "latex";
ax.YLim = [-1.2*Vp,1.2*Vp];
title(signals2.Name,"FontSize",14,'Interpreter','latex');
xlabel('Tiempo (s)','interpreter','latex','FontSize',20);
ylabel('$s(t)$','interpreter','latex','FontSize',20);
lgd = legend; lgd.Interpreter = "latex";
set(lgd, 'FontSize', 18);