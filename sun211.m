% rrs_solar_tracking_sun_sphere_HW.m
% 功能：鼠标控制 3D 太阳位置，PID 驱动 3RRS 平台实时追踪，并解算舵机角度
%      通过串口把角度发给 STM32->PCA9685（协议：S1:090\n）：

clear; clc; close all;

%% ===== 0. 硬件映射参数（你只需要调这几行）=====
servoOffset = [90 90 90];     % 舵机中位（常见 90°）
servoDir    = [ -1  -1  -1];     % 方向，不对就把对应改成 -1
servoMinMax = [0 180];        % 舵机限幅

sendPeriod  = 0.02;           % 20ms=50Hz（和你 App 一样稳）
debugPrint  = false;          % true 会在命令窗口打印发出的 Sx:yyy

%% ===== 1. 串口配置 =====
baud = 115200;

% 如果你想指定端口，写这里：例如 "COM5"；不指定就留空 ""
portPref = "";

s = [];
try
    ports = serialportlist("available");
    if isempty(ports)
        warning("未检测到可用串口，将仅进行仿真演示。");
    else
        if strlength(portPref) > 0
            port = string(portPref);
        else
            port = string(ports(1)); % 自动选第一个可用串口
        end

        s = serialport(port, baud);
        configureTerminator(s, "LF");
        flush(s);
        disp("Serial connected: " + port);
    end
catch ME
    warning("串口打开失败（可能被占用/端口不对）：%s\n将仅进行仿真演示。", ME.message);
    s = [];
end

tSend = tic;                     % 发送节流计时
lastSentAngles = [NaN NaN NaN];   % 仅变化发送

%% ===== 2. 几何参数 =====
Rb = 0.20; Rp = 0.12; h = 0.18; L1 = 0.10; L2 = 0.18; Tb = 0.02; Tp = 0.02;
baseTh = deg2rad([0 120 240]); platTh = deg2rad([0 120 240]);
B = [Rb*cos(baseTh(:)), Rb*sin(baseTh(:)), zeros(3,1)];
P_local = [Rp*cos(platTh(:)), Rp*sin(platTh(:)), zeros(3,1)];
p0 = [0 0 h];

%% ===== 3. PID 与控制参数 =====
Kp = 0.02; Ki = 0; Kd = 0;       % PID参数，可微调手感
maxTiltDeg = 35;                 % 最大追踪角度

% 状态变量（单位：deg）
curr_roll = 0; curr_pitch = 0;
err_sum_r = 0; err_sum_p = 0;
last_err_r = 0; last_err_p = 0;
dt = 0.02;

% 历史记录
historyLen = 100;
servo_hist = zeros(historyLen, 3);

% 鼠标交互目标
global target_r target_p
target_r = 0; target_p = 0;

%% ===== 4. 图形界面初始化 =====

% --- 窗口 1: 3D 仿真 ---
fig1 = figure("Name","3RRS 太阳能追踪仿真(HW)","Position",[50, 200, 600, 600]);
ax = axes(fig1); hold(ax,"on"); grid(ax,"on"); axis(ax,"equal");
view(ax, 45, 20);
xlim(ax,[-0.4 0.4]); ylim(ax,[-0.4 0.4]); zlim(ax,[0 0.55]);
xlabel(ax,"X"); ylabel(ax,"Y"); zlabel(ax,"Z");
camproj(ax, "perspective"); lighting(ax, "gouraud"); camlight(ax, "headlight"); material(ax, "dull");

% === 空中的太阳小球 ===
[XS_sun, YS_sun, ZS_sun] = sphere(15);
sunRadius = 0.025;
sunHeight = 0.45;
sunDistScale = 0.3;
hSun3D = surf(ax, XS_sun*sunRadius, YS_sun*sunRadius, ZS_sun*sunRadius + sunHeight, ...
    'FaceColor', 'y', 'EdgeColor', 'none', 'FaceLighting', 'none', 'FaceAlpha', 0.9);

% 静平台
baseV = [B; B + [0 0 -Tb]];
Ftri = [1 2 3; 4 6 5; 1 2 5; 1 5 4; 2 3 6; 2 6 5; 3 1 4; 3 4 6];
patch(ax, "Faces", Ftri, "Vertices", baseV, "FaceAlpha", 0.25, "EdgeAlpha", 0.3);

% 动平台
platPatch = patch(ax, "Faces", Ftri, "Vertices", zeros(6,3), "FaceAlpha", 0.55, "EdgeAlpha", 0.3);

% 机械臂与球头
arm1 = gobjects(3,1); arm2 = gobjects(3,1); sBall = gobjects(3,1);
[XS,YS,ZS] = sphere(10); ballR = 0.012;
for i=1:3
    arm1(i) = plot3(ax, [0 0],[0 0],[0 0], "LineWidth", 3);
    arm2(i) = plot3(ax, [0 0],[0 0],[0 0], "LineWidth", 2);
    sBall(i) = surf(ax, XS*ballR, YS*ballR, ZS*ballR, "EdgeColor","none", "FaceAlpha",0.85);
end

% --- 窗口 2: 追踪控制垫 ---
fig2 = figure("Name","光追踪控制垫 (移动鼠标)","Position",[670, 450, 350, 350]);
ax_pad = axes(fig2); set(ax_pad, 'XLim', [-1 1], 'YLim', [-1 1]);
grid on; hold on; title('鼠标位置 = 太阳方位目标');
xlabel('Pitch (前后)'); ylabel('Roll (左右)');
hSunPoint = plot(ax_pad, 0, 0, 'ro', 'MarkerSize', 12, 'MarkerFaceColor', 'y');
set(fig2, 'WindowButtonMotionFcn', @(src, event) mouseMoveCallback(src, ax_pad, hSunPoint, maxTiltDeg));

% --- 窗口 3: 舵机输出角度监控 ---
fig3 = figure("Name","各个舵机实时输出角度 (deg)","Position",[1030, 200, 350, 600]);
hServo = gobjects(3,1); colors = ['r', 'g', 'b'];
for i = 1:3
    subplot(3,1,i);
    hServo(i) = plot(zeros(historyLen, 1), 'LineWidth', 1.5, 'Color', colors(i));
    grid on; title(sprintf('电机 %d 实时角度', i)); ylim([-20, 200]);
end

%% ===== 5. 主循环 =====
q2_prev = zeros(3,1);
last_servo_cmd = servoOffset;  % ✅ 兜底：初值用中位（别用 0）

while ishandle(fig1) && ishandle(fig2) && ishandle(fig3)

    % 获取目标（来自鼠标回调）
    tr = target_r; tp = target_p;

    % 更新 3D 太阳位置
    sx = sunDistScale * sind(tp);
    sy = -sunDistScale * sind(tr);
    set(hSun3D, 'XData', XS_sun*sunRadius + sx, ...
                'YData', YS_sun*sunRadius + sy, ...
                'ZData', ZS_sun*sunRadius + sunHeight);

    % A. PID 追踪计算（单位：deg）
    err_r = tr - curr_roll; err_sum_r = err_sum_r + err_r * dt;
    out_r = Kp*err_r + Ki*err_sum_r + Kd*((err_r - last_err_r)/dt);
    last_err_r = err_r;

    err_p = tp - curr_pitch; err_sum_p = err_sum_p + err_p * dt;
    out_p = Kp*err_p + Ki*err_sum_p + Kd*((err_p - last_err_p)/dt);
    last_err_p = err_p;

    curr_roll  = curr_roll  + out_r;
    curr_pitch = curr_pitch + out_p;

    % B. 逆运动学（IK）解算与绘图
    R = eulZYX(0, curr_pitch, curr_roll);
    P = (R * P_local')' + p0;
    platPatch.Vertices = [P; P + [0 0 -Tp]];

    % ✅ 用上一次有效命令兜底（防止 continue 后变 0）
    current_servo_angles = last_servo_cmd;

    for i=1:3
        Bi = B(i,:); Pi = P(i,:); r_vec = Pi - Bi;

        theta_rad = atan2(Bi(2), Bi(1));
        ex = [cos(theta_rad), sin(theta_rad), 0];
        ez = [0 0 1];
        ey = cross(ez, ex);

        x = dot(r_vec, ex);
        z = r_vec(3);
        yperp = dot(r_vec, ey);

        C = (x*x + yperp*yperp + z*z + L1*L1 - L2*L2) / (2*L1);
        Rxz = hypot(x, z);
        if Rxz < 1e-6
            continue; % 继续用兜底角度
        end

        ratio = C / Rxz;
        ratio = max(-1, min(1, ratio));
        a = acos(ratio);
        gamma = atan2(z, x);

        q2c = chooseClosest(gamma+a, gamma-a, q2_prev(i));
        Ki_joint = Bi + L1*(cos(q2c)*ex + sin(q2c)*ez);
        if dot((Ki_joint - Bi), ex) < 0
            q2c = otherBranch(gamma+a, gamma-a, q2c);
            Ki_joint = Bi + L1*(cos(q2c)*ex + sin(q2c)*ez);
        end
        q2_prev(i) = q2c;

        % ✅ 关键修复：机构角(±) -> 舵机角(0..180)，以中位为基准
        servoDeg = servoOffset(i) + servoDir(i) * rad2deg(q2c);
        servoDeg = max(servoMinMax(1), min(servoMinMax(2), round(servoDeg)));
        current_servo_angles(i) = servoDeg;

        % 绘图
        set(arm1(i), "XData", [Bi(1) Ki_joint(1)], "YData", [Bi(2) Ki_joint(2)], "ZData", [Bi(3) Ki_joint(3)]);
        set(arm2(i), "XData", [Ki_joint(1) Pi(1)], "YData", [Ki_joint(2) Pi(2)], "ZData", [Ki_joint(3) Pi(3)]);
        set(sBall(i), "XData", XS*ballR + Pi(1), "YData", YS*ballR + Pi(2), "ZData", ZS*ballR + Pi(3));
    end

    % 更新兜底
    last_servo_cmd = current_servo_angles;

    % C. 更新舵机角度曲线
    servo_hist = [servo_hist(2:end, :); current_servo_angles];
    for i = 1:3
        set(hServo(i), 'YData', servo_hist(:, i));
    end

    % D. 发送数据给硬件（协议：S1:090\n）
    if ~isempty(s) && isvalid(s)
        if toc(tSend) >= sendPeriod
            tSend = tic;

            for id = 1:3
                a = current_servo_angles(id);

                % 仅变化发送（更稳）
                if ~isequal(lastSentAngles(id), a)
                    cmd = sprintf("S%d:%03d", id, a);
                    writeline(s, cmd);
                    lastSentAngles(id) = a;

                    if debugPrint
                        disp(cmd);
                    end
                end
            end
        end
    end

    drawnow limitrate;
end

% 退出时关串口
try
    if ~isempty(s) && isvalid(s)
        clear s
    end
catch
end

%% ===== 6. 交互回调函数 =====
function mouseMoveCallback(~, ax_pad, hSunPoint, maxTilt)
    global target_r target_p
    cp = get(ax_pad, 'CurrentPoint');
    mx = max(-1, min(1, cp(1,1)));
    my = max(-1, min(1, cp(1,2)));
    target_p = mx * maxTilt;
    target_r = my * maxTilt;
    set(hSunPoint, 'XData', mx, 'YData', my);
end

%% ===== 7. 助手函数 =====
function R = eulZYX(yaw, pitch, roll)
    cy = cosd(yaw); sy = sind(yaw);
    cp = cosd(pitch); sp = sind(pitch);
    cr = cosd(roll); sr = sind(roll);
    R = [cy -sy 0; sy cy 0; 0 0 1] * ...
        [cp 0 sp; 0 1 0; -sp 0 cp] * ...
        [1 0 0; 0 cr -sr; 0 sr cr];
end

function q = chooseClosest(q1, q2, qprev)
    q1 = mod(q1+pi, 2*pi)-pi;
    q2 = mod(q2+pi, 2*pi)-pi;
    qprev = mod(qprev+pi, 2*pi)-pi;
    if abs(q1-qprev) <= abs(q2-qprev)
        q = q1;
    else
        q = q2;
    end
end

function q = otherBranch(q1, q2, qcur)
    q1 = mod(q1+pi, 2*pi)-pi;
    q2 = mod(q2+pi, 2*pi)-pi;
    qcur = mod(qcur+pi, 2*pi)-pi;
    if abs(qcur-q1) < abs(qcur-q2)
        q = q2;
    else
        q = q1;
    end
end
