

clear; clc; close all;

%% ---------------- 1. Simulation time settings ----------------
dt       = 0.5;
T_final  = 3600;
t        = 0:dt:T_final-dt;
N        = length(t);

%% ---------------- 2. Battery thermal model parameters ----------------
m      = 25;
cp     = 900;
R_int  = 0.015;
T_amb  = 30;
T_set  = 32;
T0     = 34;

%% ---------------- 3. Cooling model parameters ----------
h0      = 0.5;
k_fan   = 0.06;
k_pump  = 0.09;

%% ---------------- 4. Actuator (DC motor) dynamics -----------------
tau_fan  = 4;   K_fan  = 25;
tau_pump = 6;   K_pump = 15;
V_max    = 12;

%% ---------------- 5. PID controller gains --------------------------
Kp = 1.8;
Ki = 0.015;
Kd = 3.0;

%% ---------------- 6. Disturbance: synthetic EV driving current -----
rng(7);
I_base  = 60 + 40*sin(2*pi*t/900) + 25*sin(2*pi*t/180);
I_noise = 8*randn(1,N);
I_profile = min(max(I_base + I_noise, 5), 220);

%% ---------------- 7. Pre-allocate state arrays ----------------------
T_batt     = zeros(1,N);  T_batt(1) = T0;
omega_fan  = zeros(1,N);
omega_pump = zeros(1,N);
V_ctrl     = zeros(1,N);
integral   = 0;
prev_err   = 0;

%% ---------------- 8. Main simulation loop (explicit Euler) ----------
for k = 2:N
    err   = T_batt(k-1) - T_set;
    integral = integral + err*dt;
    deriv = (err - prev_err)/dt;

    u = Kp*err + Ki*integral + Kd*deriv;
    u_sat = min(max(u, 0), V_max);

    if u ~= u_sat
        integral = integral - err*dt;
    end
    V_ctrl(k) = u_sat;

    omega_fan(k)  = omega_fan(k-1)  + dt/tau_fan *(-omega_fan(k-1)  + K_fan*u_sat);
    omega_pump(k) = omega_pump(k-1) + dt/tau_pump*(-omega_pump(k-1) + K_pump*u_sat);

    Q_gen = (I_profile(k)^2) * R_int;
    Q_rem = (h0 + k_fan*omega_fan(k) + k_pump*omega_pump(k)) * (T_batt(k-1) - T_amb);
    dT    = (Q_gen - Q_rem) / (m*cp);

    T_batt(k) = T_batt(k-1) + dt*dT;
    prev_err  = err;
end

%% ---------------- 9. Performance metrics -----------------------------
overshoot   = max(T_batt) - T_set;
settle_band = 0.5;
settle_idx  = find(abs(T_batt - T_set) > settle_band, 1, 'last');
if isempty(settle_idx); settle_idx = 1; end
settle_time = t(settle_idx);

fprintf('---- Performance summary ----\n');
fprintf('Max battery temperature : %.2f C\n', max(T_batt));
fprintf('Min battery temperature : %.2f C\n', min(T_batt));
fprintf('Max overshoot above set : %.2f C\n', overshoot);
fprintf('Approx settling time    : %.1f s\n', settle_time);

%% ---------------- 10. Plot results ------------------------------------
figure('Name','EV Battery Thermal Management - Simulation Results','Color','w');

subplot(3,1,1);
plot(t/60, T_batt, 'r', 'LineWidth', 1.5); hold on;
yline(T_set, '--k', 'Setpoint');
xlabel('Time (min)'); ylabel('Temp (\circC)');
title('Battery Temperature Control');
legend('Battery Temp','Setpoint','Location','best'); grid on;

subplot(3,1,2);
plot(t/60, omega_fan, 'b', 'LineWidth', 1.3); hold on;
plot(t/60, omega_pump, 'g', 'LineWidth', 1.3);
xlabel('Time (min)'); ylabel('Speed (rad/s)');
title('Fan & Pump Motor Speed (Actuator Response)');
legend('Fan speed','Pump speed','Location','best'); grid on;

subplot(3,1,3);
plot(t/60, I_profile, 'm', 'LineWidth', 1);
xlabel('Time (min)'); ylabel('Current (A)');
title('Simulated EV Driving Load Current (Disturbance Input)');
grid on;

sgtitle('Automatic Electromechanical BTMS - Full Simulation');

if ~exist('../results','dir'); mkdir('../results'); end
saveas(gcf, '../results/simulation_result.png');

%% ---------------- ---------------
save('../results/simulation_data.mat', 't','T_batt','omega_fan','omega_pump','I_profile','V_ctrl');
disp('Simulation complete. Results saved in /results folder.');