% File: plot_voltage_vs_energy_avg.m

% 读取 CSV 文件
data = readtable('tilt_log_kp3000.csv');

% 提取列
Tilt = data.TILT;
Time = data.TIME;

% 时间间隔
dt = 0.01; % 100 毫秒


% 绘图
figure;
plot(Time, Tilt, 'LineWidth', 1.5);
xlabel('Time');
ylabel('TiltAngle');
title('Tilt Angle Tuning Plot');
grid on;
