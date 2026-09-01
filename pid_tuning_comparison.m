
clear; clc; close all;

dt = 0.5; T_final = 3600; t = 0:dt:T_final-dt; N = length(t);

% Same driving-cycle disturbance for a fair comparison
rng(7);
I_base  = 60 + 40*sin(2*pi*t/900) + 25*sin(2*pi*t/180);
I_noise = 8*randn(1,N);
I_profile = min(max(I_base + I_noise, 5), 220);

% Three PID gain sets to compare
gainSets = [ ...
    0.6,  0.005, 0.5;   % 1: soft / under-tuned
    1.8,  0.015, 3.0;   % 2: balanced (same as main_simulation.m)
    4.0,  0.06,  8.0];  % 3: aggressive / over-tuned

labels = {'Under-tuned (soft)', 'Balanced (recommended)', 'Aggressive (over-tuned)'};
colors = {'b','g','r'};

figure('Name','PID Tuning Comparison','Color','w'); hold on;
for g = 1:size(gainSets,1)
    Kp = gainSets(g,1); Ki = gainSets(g,2); Kd = gainSets(g,3);
    T_batt = runBTMS(t, dt, N, I_profile, Kp, Ki, Kd);
    plot(t/60, T_batt, colors{g}, 'LineWidth', 1.4, 'DisplayName', labels{g});

    ov = max(T_batt) - 32;
    fprintf('%-25s -> max overshoot = %.2f C, final temp = %.2f C\n', ...
        labels{g}, ov, T_batt(end));
end
yline(32, '--k', 'Setpoint', 'DisplayName','Setpoint');
xlabel('Time (min)'); ylabel('Battery Temp (\circC)');
title('Effect of PID Tuning on Battery Temperature Control');
legend('show','Location','best'); grid on;

if ~exist('../results','dir'); mkdir('../results'); end
saveas(gcf, '../results/pid_tuning_comparison.png');
disp('PID comparison complete. Plot saved in /results folder.');

%% ---------------- helper function: reusable plant simulation ----------
function T_batt = runBTMS(t, dt, N, I_profile, Kp, Ki, Kd)
    m = 25; cp = 900; R_int = 0.015; T_amb = 30; T_set = 32; T0 = 34;
    h0 = 0.5; k_fan = 0.06; k_pump = 0.09;
    tau_fan = 4; K_fan = 25; tau_pump = 6; K_pump = 15; V_max = 12;

    T_batt = zeros(1,N); T_batt(1) = T0;
    omega_fan = zeros(1,N); omega_pump = zeros(1,N);
    integral = 0; prev_err = 0;

    for k = 2:N
        err = T_batt(k-1) - T_set;
        integral = integral + err*dt;
        deriv = (err - prev_err)/dt;
        u = Kp*err + Ki*integral + Kd*deriv;
        u_sat = min(max(u,0), V_max);
        if u ~= u_sat
            integral = integral - err*dt;
        end
        omega_fan(k)  = omega_fan(k-1)  + dt/tau_fan *(-omega_fan(k-1)  + K_fan*u_sat);
        omega_pump(k) = omega_pump(k-1) + dt/tau_pump*(-omega_pump(k-1) + K_pump*u_sat);
        Q_gen = (I_profile(k)^2)*R_int;
        Q_rem = (h0 + k_fan*omega_fan(k) + k_pump*omega_pump(k))*(T_batt(k-1)-T_amb);
        T_batt(k) = T_batt(k-1) + dt*(Q_gen - Q_rem)/(m*cp);
        prev_err = err;
    end
end