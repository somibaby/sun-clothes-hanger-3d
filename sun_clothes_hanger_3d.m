%% =======================================================================
%  SOLAR TELESCOPING CLOTHES HANGER  -  MATLAB SIMULATION
%  =======================================================================
%  WHAT IT DOES
%    A wall-mounted hanger with two telescoping arms. A sun sensor on top
%    decides when to push the clothes OUT from under the roof into the sun,
%    and when to pull them back IN (dark or rain).
%
%  HOW THE SYSTEM IS BUILT  (same order as the code below)
%
%     [1] SENSORS  --->  [2] CONTROLLER  --->  [3] MOTOR + ARMS  --->  [4] RESULTS / 3D
%     light (lux)        state machine         moves the arms         plots + animation
%     rain (yes/no)      decides IN / OUT      stops at end switches
%
%  KEY IDEAS TO EXPLAIN IN AN INTERVIEW
%    - State machine : 4 states  RETRACTED -> EXTENDING -> EXTENDED -> RETRACTING
%    - Hysteresis    : go OUT above 20 000 lux, come IN below 8 000 lux,
%                      so a passing cloud does not make it move back and forth
%    - Debounce      : the condition must last 60 s before the arms move
%    - Safety        : rain overrides everything and retracts immediately
%    - Limit switches: the motor stops at the fully-in and fully-out ends
%
%  FILES : keep  solar-telescoping-hanger.obj  in the same folder.
%  =======================================================================

clear; clc; close all;

%% STEP 0 - SETTINGS  (all numbers you may want to change are in one place)
P = settings();

%% STEP 1 - SENSORS   (simulate one day of light and rain)
[t, lux, rain] = makeSensorData(P);

%% STEP 2 - CONTROLLER + MOTOR   (decide IN/OUT every second, move the arms)
[pos, state] = runController(t, lux, rain, P);

%% STEP 3 - RESULTS   (graphs + list of what happened when)
plotResults(t, lux, rain, pos, state, P);
printEventLog(t, lux, rain, state);

%% STEP 4 - 3D ANIMATION   (show it on the real 3D model)
if P.animate
    model = loadModel(P.objFile);
    scene = buildScene(model);
    playAnimation(scene, t, pos, lux, rain, state, P);
end


%% #######################################################################
%  FUNCTIONS  -  each one does ONE job. Grouped in the same order as above.
%  #######################################################################

%% ===================== GROUP A : SETTINGS ===============================

function P = settings()
    % --- simulation time
    P.dt        = 1;            % time step [s]
    P.tEnd      = 24*3600;      % one full day [s]

    % --- sensor thresholds (hysteresis band)
    P.luxOn     = 20000;        % above this = "sun detected" -> go OUT  [lux]
    P.luxOff    = 8000;         % below this = "too dark"     -> come IN [lux]
    P.debounce  = 60;           % condition must hold this long [s]

    % --- motor + arms (from the 3D model)
    P.travel    = 0.66;         % total arm stroke: 2 stages x 0.33 m [m]
    P.speed     = 0.022;        % arm speed [m/s]  (~30 s for full stroke)

    % --- 3D animation
    P.objFile    = 'solar-telescoping-hanger.obj';
    P.animate    = true;        % false = graphs only
    P.frameDelay = 0.08;        % pause per frame [s]: bigger = slower
    P.animStep   = 120;         % frame every 120 s when nothing moves
    P.moveStep   = 1;           % frame every 1 s while the arms move
end

%% ===================== GROUP B : SENSORS ================================

function [t, lux, rain] = makeSensorData(P)
    % Fake but realistic day: sunrise 06:00, sunset 19:00,
    % one passing cloud (tests hysteresis) and one rain shower (tests safety).
    rng(1);                                    % same "random" noise every run
    t = 0:P.dt:P.tEnd;
    h = t/3600;                                % time in hours

    sun   = 100000 * max(0, sin(pi*(h-6)/13)); % clear-sky light curve [lux]
    cloud = ones(size(h));
    cloud(h >= 10.5 & h < 11.2) = 0.15;        % cloud at 10:30-11:12

    rain = (h >= 14 & h < 14.75);              % rain at 14:00-14:45
    cloud(rain) = 0.1;                         % dark while raining

    noise = 800*randn(size(h));                % sensor noise
    lux   = max(sun.*cloud + noise, 0);
end

%% ===================== GROUP C : CONTROLLER + MOTOR =====================

function [pos, state] = runController(t, lux, rain, P)
    % Main control loop: runs once per second for the whole day.
    N     = numel(t);
    pos   = zeros(1,N);                        % arm extension [m]
    state = strings(1,N);                      % controller state

    S = "RETRACTED"; timer = 0; x = 0;         % start: arms in, timer reset

    for k = 1:N
        [S, u, timer] = decideState(S, timer, lux(k), rain(k), P);  % brain
        x = moveArm(x, u, P);                                       % muscle
        S = checkLimitSwitches(S, x, P);                            % end stops
        pos(k) = x;  state(k) = S;
    end
end

function [S, u, timer] = decideState(S, timer, lux, rain, P)
    % THE STATE MACHINE.  u = motor command: +1 out, -1 in, 0 stop.
    sunny = lux > P.luxOn && ~rain;
    dark  = lux < P.luxOff;

    switch S
        case "RETRACTED"                       % waiting under the roof
            u = 0;
            timer = countIf(sunny, timer, P);
            if timer >= P.debounce, S = "EXTENDING"; timer = 0; end

        case "EXTENDING"                       % motor pushing arms out
            u = +1;
            if rain, S = "RETRACTING"; end     % rain: abort, go back in

        case "EXTENDED"                        % drying in the sun
            u = 0;
            if rain
                S = "RETRACTING"; timer = 0;   % rain: no waiting
            else
                timer = countIf(dark, timer, P);
                if timer >= P.debounce, S = "RETRACTING"; timer = 0; end
            end

        case "RETRACTING"                      % motor pulling arms in
            u = -1;
    end
end

function timer = countIf(condition, timer, P)
    % Debounce timer: counts up while the condition is true, resets otherwise.
    if condition, timer = timer + P.dt; else, timer = 0; end
end

function x = moveArm(x, u, P)
    % Motor model: constant speed, cannot go past the ends of the rail.
    x = x + u * P.speed * P.dt;
    x = min(max(x, 0), P.travel);
end

function S = checkLimitSwitches(S, x, P)
    % Limit switches tell the controller the arms reached the end.
    if S == "EXTENDING"  && x >= P.travel, S = "EXTENDED";  end
    if S == "RETRACTING" && x <= 0,        S = "RETRACTED"; end
end

%% ===================== GROUP D : RESULTS ================================

function plotResults(t, lux, rain, pos, state, P)
    % 4 graphs stacked: light, rain, arm position, controller state.
    h = t/3600;
    names = ["RETRACTED","EXTENDING","EXTENDED","RETRACTING"];
    [~, sNum] = ismember(state, names);

    figure('Name','Solar hanger - results','Color','w','Position',[80 80 900 750]);

    subplot(4,1,1);
    plot(h, lux/1000, 'Color',[0.9 0.6 0]); hold on;
    yline(P.luxOn/1000,'--g','extend'); yline(P.luxOff/1000,'--r','retract');
    ylabel('Light [klux]'); title('Sun sensor'); grid on; xlim([0 24]);

    subplot(4,1,2);
    area(h, double(rain), 'FaceColor',[0.3 0.5 1], 'EdgeColor','none');
    ylim([0 1.2]); yticks([0 1]); yticklabels({'dry','rain'});
    ylabel('Rain'); title('Rain sensor'); grid on; xlim([0 24]);

    subplot(4,1,3);
    plot(h, pos*100, 'k', 'LineWidth',1.5);
    ylim([-5 P.travel*100+5]); xlim([0 24]);
    ylabel('Extension [cm]'); title('Arm extension (0 = under roof)'); grid on;

    subplot(4,1,4);
    stairs(h, sNum, 'LineWidth',1.5);
    yticks(1:4); yticklabels(names); ylim([0.5 4.5]); xlim([0 24]);
    xlabel('Time of day [h]'); title('Controller state'); grid on;
end

function printEventLog(t, lux, rain, state)
    % Prints every moment the state changed, e.g. "06:54  EXTENDING".
    fprintf('\n--- Event log ---\n');
    changes = [1, find(state(2:end) ~= state(1:end-1)) + 1];
    for k = changes
        fprintf('%s  %-11s (light %6.0f lux, rain %d)\n', ...
            fmtTime(t(k)), state(k), lux(k), rain(k));
    end
end

function s = fmtTime(sec)
    s = sprintf('%02d:%02d', floor(sec/3600), floor(mod(sec,3600)/60));
end

%% ===================== GROUP E : LOAD THE 3D MODEL ======================

function M = loadModel(fileName)
    % Find the .obj file, read it, print a short summary.
    path = findFile(fileName);
    M = readObj(path);
    fprintf('\nLoaded %s: %d parts, %d triangles\n', path, numel(M.name), M.nFaces);
end

function p = findFile(name)
    % Look next to this script first; if not found, ask the user to pick it.
    here  = fileparts(mfilename('fullpath'));
    cands = {name, fullfile(here, name), fullfile(pwd, name)};
    for i = 1:numel(cands)
        if isfile(cands{i}), p = cands{i}; return; end
    end
    [f, d] = uigetfile('*.obj', 'Select solar-telescoping-hanger.obj');
    if isequal(f, 0), error('OBJ model not found.'); end
    p = fullfile(d, f);
end

function M = readObj(file)
    % Reads an OBJ file: vertices (v), triangles (f), part names (o), materials (usemtl).
    L   = splitlines(fileread(file));
    isV = startsWith(L, 'v ');   isF = startsWith(L, 'f ');
    isO = startsWith(L, 'o ');   isM = startsWith(L, 'usemtl ');

    V = sscanf(strjoin(extractAfter(L(isV), 2)', ' '), '%f');
    V = reshape(V, 3, []).';                              % one row per vertex

    fl = regexprep(extractAfter(L(isF), 2), '/[^\s]*', ''); % keep vertex index only
    F  = sscanf(strjoin(fl', ' '), '%d');
    if numel(F) ~= 3*nnz(isF)
        error('Only triangulated OBJ files are supported.');
    end
    F = reshape(F, 3, []).';                              % one row per triangle

    partOfLine = cumsum(isO);                             % which part each line belongs to
    partOfFace = partOfLine(isF);
    names = strtrim(extractAfter(L(isO), 2));
    mats  = repmat({'default'}, numel(names), 1);
    for i = find(isM)'
        o = partOfLine(i); if o > 0, mats{o} = strtrim(extractAfter(L{i}, 7)); end
    end

    n = numel(names);
    M.name = names;  M.mat = mats;  M.V = cell(n,1);  M.F = cell(n,1);
    for o = 1:n                                           % split into separate parts
        fo = F(partOfFace == o, :);
        [u, ~, j] = unique(fo(:));
        M.V{o} = V(u, [1 3 2]);                           % OBJ is Y-up -> MATLAB Z-up
        M.F{o} = reshape(j, [], 3);
    end
    M.nFaces = size(F,1);
    M.color  = materialColors();
end

function kd = materialColors()
    % One fixed colour per material name used in the OBJ file.
    kd = struct('shell_white',[0.94 0.94 0.92], 'rubber_grey',[0.22 0.22 0.24], ...
        'solar_cell',[0.07 0.11 0.32], 'aluminum',[0.74 0.76 0.80], ...
        'sensor_dome',[0.85 0.93 1.0], 'sensor_amber',[1.0 0.6 0.1], ...
        'beech_hanger',[0.86 0.68 0.45], 'default',[0.7 0.7 0.7]);
end

%% ===================== GROUP F : BUILD THE 3D SCENE =====================
%  Scene axes:  X = along the wall,  Y = outward from the wall,  Z = up.

function S = buildScene(M)
    % Creates the window, the house and the model, and sets the camera.
    S.f  = figure('Name','Solar telescoping hanger - 3D', ...
                  'Color',[0.55 0.75 0.95], 'Position',[120 120 1000 700]);
    S.ax = axes(S.f, 'Color','none', 'Clipping','off');
    hold(S.ax,'on'); axis(S.ax,'equal'); axis(S.ax,'off');
    axis(S.ax, [-0.75 0.75 -0.62 0.75 -0.55 0.40]);

    drawHouse(S.ax);
    S = drawModelParts(S, M);                       % adds moving groups to S
    S = addLightsAndHud(S);
    setCamera(S.ax);
end

function drawHouse(ax)
    % Wall, roof eave and grass (not part of the OBJ model).
    boxPatch(ax, [-0.75 0.75], [-0.62 -0.595], [-0.55 0.34],  [0.86 0.80 0.70]); % wall
    boxPatch(ax, [-0.72 0.72], [-0.62 -0.08],  [0.30 0.325],  [0.55 0.30 0.20]); % roof
    boxPatch(ax, [-0.72 0.72], [-0.10 -0.08],  [0.27 0.325],  [0.45 0.25 0.17]); % fascia
    patch(ax, [-0.75 0.75 0.75 -0.75], [-0.62 -0.62 0.75 0.75], -0.55*[1 1 1 1], ...
          [0.45 0.62 0.35], 'EdgeColor','none');                              % grass
    text(ax, 0, -0.35, 0.36, 'ROOF', 'HorizontalAlignment','center', ...
         'FontWeight','bold', 'Color',[0.35 0.18 0.1]);
end

function S = drawModelParts(S, M)
    % Draws every part of the OBJ and attaches it to the group that moves it:
    %   fixed   : housing, panel, sensor, LED, tube 1
    %   stage 2 : tube 2 + collar 2          (slides out of tube 1)
    %   stage 3 : tube 3 + tip + crossbar    (slides out of tube 2)
    %   hanger i: hook + shoulders + bar     (spreads along the arm)
    S.gS2 = hgtransform('Parent',S.ax);
    S.gS3 = hgtransform('Parent',S.ax);
    S.gH  = gobjects(1,4);
    for i = 1:4, S.gH(i) = hgtransform('Parent',S.ax); end

    S.zExt = [-0.40 -0.15 0.10 0.37];       % hanger positions when arms are OUT
    S.zRet = [-0.45 -0.39 -0.33 -0.27];     % hanger positions when arms are IN

    for o = 1:numel(M.name)
        name = M.name{o};
        parent = whichGroup(S, name);
        c = partColor(M, o);
        alpha = 1; if strcmp(M.mat{o}, 'sensor_dome'), alpha = 0.35; end
        patch('Parent',parent, 'Vertices',M.V{o}, 'Faces',M.F{o}, ...
              'FaceColor',c, 'EdgeColor','none', 'FaceAlpha',alpha, ...
              'FaceLighting','flat', 'AmbientStrength',0.45, 'SpecularStrength',0.3);
    end
end

function parent = whichGroup(S, name)
    % Decides which moving group a model part belongs to (by its name).
    parent = S.ax;                                                 % fixed
    if ~isempty(regexp(name, '^arm_[LR]_(tube2|collar2)$', 'once'))
        parent = S.gS2;
    elseif ~isempty(regexp(name, '^arm_[LR]_(tube3|tip)$', 'once')) || strcmp(name,'crossbar')
        parent = S.gS3;
    else
        tok = regexp(name, '^hanger_[LR](\d)_', 'tokens', 'once');
        if ~isempty(tok), parent = S.gH(str2double(tok{1})); end
    end
end

function c = partColor(M, o)
    key = matlab.lang.makeValidName(M.mat{o});
    if isfield(M.color, key), c = M.color.(key); else, c = M.color.default; end
end

function S = addLightsAndHud(S)
    S.sunLight = light(S.ax, 'Style','infinite', 'Position',[0 1 1]);
    camlight(S.ax, 'headlight'); lighting(S.ax, 'flat');
    S.sunDot = plot3(S.ax, 0.6, 0.6, 0.38, 'o', 'MarkerSize',26, ...
                     'MarkerFaceColor',[1 0.85 0], 'MarkerEdgeColor','none');
    S.hud = title(S.ax, '', 'FontSize',13, 'FontName','Menlo');
end

function boxPatch(ax, xr, yr, zr, c)
    % Draws a solid box between the given x, y, z ranges.
    [X, Y, Z] = ndgrid(xr, yr, zr);
    v  = [X(:) Y(:) Z(:)];
    fc = [1 2 4 3; 5 6 8 7; 1 2 6 5; 3 4 8 7; 1 3 7 5; 2 4 8 6];
    patch(ax, 'Vertices',v, 'Faces',fc, 'FaceColor',c, 'EdgeColor','none', ...
          'FaceLighting','flat');
end

%% ===================== GROUP G : PLAY THE ANIMATION =====================

function playAnimation(S, t, pos, lux, rain, state, P)
    % Loops through the chosen time steps and updates the picture.
    frames = chooseFrames(t, state, P);
    for k = frames
        if ~isvalid(S.f), return; end              % window was closed
        moveArms(S, pos(k) / P.travel);            % 0 = in, 1 = out
        updateSky(S, t(k), lux(k), rain(k));
        updateHud(S, t(k), lux(k), state(k), pos(k), rain(k));
        drawnow limitrate;
        pause(P.frameDelay);                       % controls the speed
    end
end

function frames = chooseFrames(t, state, P)
    % Few frames when idle (every 2 min), many while the arms move (every 1 s).
    moving = state == "EXTENDING" | state == "RETRACTING";
    frames = unique([1:P.animStep:numel(t), ...
                     find(moving & mod(t, P.moveStep) == 0), ...
                     find([false, diff(moving) ~= 0])]);
end

function moveArms(S, s)
    % s = 0 (fully in) ... 1 (fully out). Each stage slides 0.33 m.
    S.gS2.Matrix = makehgtform('translate', [0 -0.33*(1-s) 0]);
    S.gS3.Matrix = makehgtform('translate', [0 -0.66*(1-s) 0]);
    for i = 1:4
        S.gH(i).Matrix = makehgtform('translate', [0 (S.zRet(i)-S.zExt(i))*(1-s) 0]);
    end
end

function updateSky(S, tk, lux, rain)
    % Sky colour follows the light level; sun moves across the sky.
    day = min(lux/60000, 1);
    S.f.Color = [0.08 0.10 0.22]*(1-day) + [0.55 0.75 0.95]*day;
    if rain, S.f.Color = [0.45 0.50 0.58]; end

    hr = tk/3600;
    el = sin(pi*(hr-6)/13);                        % sun height (0 at sunrise/sunset)
    if el > 0
        az = pi*(hr-6)/13;
        S.sunLight.Position = [cos(az) 0.6 max(el,0.05)];
        S.sunLight.Color    = [1 1 1]*max(0.25, day);
        S.sunDot.Visible = 'on';
        S.sunDot.XData = 0.7*cos(az);  S.sunDot.ZData = 0.1 + 0.3*el;
    else
        S.sunLight.Color = [0.15 0.15 0.25];  S.sunDot.Visible = 'off';
    end
end

function updateHud(S, tk, lux, st, x, rain)
    % Text line at the top: time | light | state | extension.
    rs = ''; if rain, rs = '  RAIN'; end
    S.hud.String = sprintf('%s  |  %6.0f lux  |  %-10s  |  %2.0f cm%s', ...
                           fmtTime(tk), lux, st, x*100, rs);
    S.hud.Color = [1 1 1]*(lux/60000 < 0.4);       % white text at night
end

%% ===================== GROUP H : CAMERA ================================

function setCamera(ax)
    % Fixed side view, looking along the wall, so the arms slide left-right.
    view(ax, 90, 6);                   % azimuth 90 deg (from the side), 6 deg up
    camproj(ax, 'orthographic');       % flat view, no perspective distortion
    camtarget(ax, [0 -0.05 -0.05]);    % aim at the middle of the unit
    camup(ax, [0 0 1]);                % Z is up
    ax.CameraViewAngle = 7;            % zoom
end
