clear; clc; close all;

%% -------------------- Load fuzzy inference systems --------------------
load('oil 2mm_FCM_fis_X.mat');  % -> fis_X
load('oil 2mm_FCM_fis_F.mat');  % -> fis_F
assert(exist('fis_X','var')==1 && exist('fis_F','var')==1, 'fis_X or fis_F not found.');

%% -------------------- TCP to Python --------------------
ip   = '127.0.0.1';
port = 65432;
t = tcpclient(ip, port);
fprintf('[TCP] Connected to Python at %s:%d\n', ip, port);

%% -------------------- Detection/DAQ parameters --------------------
threshold   = 0.05;     % V spike threshold (relative to baseline)
time_span   = 5.0;      % s capture window after trigger (includes trigger chunk)
detect_dt   = 0.01;     % s polling chunk
use_abs     = false;    % set true if negative spikes also matter

%% -------------------- NI-DAQ setup (ai1, ai3) --------------------
dq = daq("ni");
dq.Rate = 1000;                         % keep high-rate sampling
addinput(dq, "Dev1", "ai1", "Voltage");
addinput(dq, "Dev1", "ai3", "Voltage");

fprintf("Measuring initial baseline voltages...\n");
baseline_data = read(dq, seconds(1), "OutputFormat", "Matrix");
baseline_ai1  = mean(baseline_data(:,1));
baseline_ai3  = mean(baseline_data(:,2));
fprintf("Baseline values - ai1: %f, ai3: %f\n", baseline_ai1, baseline_ai3);

%% -------------------- Figure (PNG background + marker) --------------------
fig = figure('Name','Position+Force Monitor','NumberTitle','off'); clf;
img = imread('sensor_top view.png');           % PNG background
imagesc([0 100], [0 20], img);                 % same look as your ref
axis xy equal tight;
set(gca, 'YDir','normal', 'XTick',[], 'YTick',[], 'XColor',[0.5 0.5 0.5]);
set(fig, 'Units','normalized','OuterPosition',[0 0 1 1]); % fill window
hold on;

% Large red filled marker for X (Y fixed at 10 just for visualization)
hPoint = plot(nan, nan, 'o', ...
    'MarkerSize', 30, ...
    'LineWidth', 2, ...
    'MarkerFaceColor', 'r', ...
    'MarkerEdgeColor', 'k');

% Text overlays for X and F
hx = text(2, 18.5, 'X: -- mm', 'FontSize', 16, 'FontWeight','bold', 'Color','w', 'Interpreter','none');
hf = text(2, 16.5, 'F: -- mm', 'FontSize', 16, 'FontWeight','bold', 'Color','w', 'Interpreter','none');
drawnow;

%% -------------------- MP4 video writer --------------------
ts = datestr(now,'yyyymmdd_HHMMSS');
mp4_name = ['monitor_XF_', ts, '.mp4'];
vw = VideoWriter(mp4_name, 'MPEG-4');  % H.264
vw.FrameRate = 20;
vw.Quality   = 95;
open(vw);
cleanupObj = onCleanup(@() localCleanup(vw, t));

fprintf("Monitoring started...\n");

try
    while true
        %% 1) Poll for trigger with short chunk
        detect_chunk = read(dq, seconds(detect_dt), "OutputFormat", "Matrix");
        ai1_live = detect_chunk(:,1) - baseline_ai1;
        ai3_live = detect_chunk(:,2) - baseline_ai3;

        if use_abs
            trig = any(abs(ai1_live) > threshold) || any(abs(ai3_live) > threshold);
        else
            trig = any(ai1_live > threshold) || any(ai3_live > threshold);
        end

        if trig
            fprintf("Increase detected! Recording for %.2f s...\n", time_span);

            % 2) Capture rest of the window (keep trigger chunk)
            t_remain  = max(time_span - detect_dt, 0);
            post_data = read(dq, seconds(t_remain), "OutputFormat", "Matrix");
            recorded  = [detect_chunk; post_data];

            % 3) Baseline correction
            ai1 = recorded(:,1) - baseline_ai1;
            ai3 = recorded(:,2) - baseline_ai3;

            % 4) Features (scale *5 like your originals)
            if use_abs
                pl = max(abs(ai1)) * 5;
                pr = max(abs(ai3)) * 5;
            else
                pl = max(ai1) * 5;
                pr = max(ai3) * 5;
            end

            % 5) FIS predictions
            xtest = evalfis([pl, pr], fis_X);   % mm
            ftest = evalfis([pl, pr], fis_F);   % mm

            % 6) Convert and clamp for TCP (same semantics as your original)
            xtest_m = xtest / 1000;             % meters
            z_robot = min(5*ftest / 1000, 0.1);  % meters, clamp to 100 mm downward

            % 7) Send "X,F" to Python (meters)
            msg = sprintf('%.4f,%.4f', xtest_m, z_robot);
            write(t, uint8(msg));
            fprintf("[TCP] Sent to Python: %s\n", msg);

            % 8) Update visualization and save a short hold in the MP4
            if isgraphics(hPoint)
                set(hPoint, 'XData', xtest, 'YData', 10);
            end
            set(hx, 'String', sprintf('X: %.2f mm', xtest));
            set(hf, 'String', sprintf('F: %.2f mm', ftest));
            drawnow;

            n_hold = round(0.5 * vw.FrameRate);  % ~0.5s visible
            frame = getframe(fig);
            for k = 1:n_hold
                writeVideo(vw, frame);
            end

            % 9) Re-baseline to track drift
            fprintf("Re-measuring baseline...\n");
            baseline_data = read(dq, seconds(1), "OutputFormat", "Matrix");
            baseline_ai1  = mean(baseline_data(:,1));
            baseline_ai3  = mean(baseline_data(:,2));
            fprintf("New baseline - ai1: %f, ai3: %f\n", baseline_ai1, baseline_ai3);
            fprintf("Waiting for the next detection...\n");
        else
            % Optional idle frames for continuous video (remove if not desired)
            writeVideo(vw, getframe(fig));
        end
    end

catch ME
    fprintf("\nStopping due to: %s\n", ME.message);
end

%% -------------------- Local cleanup function --------------------
function localCleanup(vw, t)
    try
        if ~isempty(vw) && isvalid(vw)
            try close(vw); catch, end
            fprintf("MP4 saved.\n");
        end
    end
    try
        if ~isempty(t); clear t; end
    end
end
