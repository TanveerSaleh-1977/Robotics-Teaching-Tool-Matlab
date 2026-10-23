%% ========================================================================
%  DENAVIT-HARTENBERG (DH) FRAMES - A DYNAMIC ROBOT GENERATOR
%  One self-contained MATLAB script. Run it top to bottom (press Run / F5).
%  It also compiles into a standalone Windows app (no MATLAB needed on the
%  other machine): see the "standalone" folder and build_exe.m.
% ========================================================================
%
%  WHAT IT DOES
%  ------------
%  Teaches *standard (distal) Denavit-Hartenberg* frame assignment for robots
%  with 2, 3 or 4 degrees of freedom.
%
%     1. A POP-UP FORM asks you for the robot: the joint sequence, the link
%        lengths, optional joint twists and optional base offsets.
%     2. The script generates the DH parameter table (alpha, a, d, theta).
%     3. It derives the Homogeneous Transformation Matrices (HTMs) for every
%        joint SYMBOLICALLY - in general form, i.e. with a1, a2 ... (link
%        lengths), d1, d2 ... (offsets, where needed), alpha1, alpha2 ...
%        (twists, where needed) and theta1, theta2 ... (joint variables) -
%        plus the composed T(0 -> N). Angle sums use the standard kinematics
%        shorthand - C12 = cos(theta1 + theta2), S123 = sin(theta1 + theta2 +
%        theta3), ... - and a legend is printed next to the matrices. Each
%        symbolic matrix is shown NEXT TO its numeric value (2 decimal places)
%        in the control window.
%     4. It builds the kinematic chain with the Robotics System Toolbox and
%        gives every link a PHYSICAL body (a cylinder of the given length, a
%        sphere at each joint, a rail at each prismatic joint).
%     5. It opens a control window with, for every joint and every link:
%           [ label ]  [ slider ]  [ editable text box ]
%        Sliders are in DEGREES (revolute) and MILLIMETRES (prismatic joints
%        and link lengths). Drag a slider for continuous animation, or type a
%        value in a box and press Enter for a smooth animation.
%        A PRISMATIC joint slides BOTH ways (q = -stroke .. +stroke), so a
%        SCARA lift can go down as well as up; the P row of the length block is
%        the stroke s_i (travel each way), not a link.
%     6. On every change it redraws the robot (with the XYZ frame of every
%        joint), the live numeric HTM, the end position and every frame
%        origin, and it keeps the symbolic matrices beside the numeric ones.
%        The end position is read off dashed projections of its x, y and z
%        components onto the three coordinate axes.
%        A base offset d_i is drawn as a PEDESTAL COLUMN along the joint axis,
%        with the link starting at the top of it - so a raised base looks like
%        a column carrying the arm, not like a link bolted to the floor.
%     7. The NEW ROBOT button (next to 3D) opens the form again, pre-filled
%        with the robot on screen, and builds the new robot in the same
%        session. The whole window is built by dhLaunchApp(cfg), so nothing
%        has to be restarted.
%
%  PLANAR vs SPATIAL
%  -----------------
%  A robot is *planar* (all joint axes parallel) when every DH twist alpha_i
%  is a multiple of 180 deg. Give a joint a 90 deg twist and the arm becomes
%  *spatial* (3-D). In the pop-up form, either type the twists (e.g. "90 0 0")
%  or tick the "spatial" box, which twists joint 1 by 90 deg for you.
%
%  SMOOTH ANIMATION (and why the control window does not call show() per frame)
%  ---------------------------------------------------------------------------
%  The control window creates every drawing object ONCE - link cylinders, the
%  slide rails, the joint spheres, a thick XYZ frame at every joint, the end
%  marker and its label - and then only rewrites their vertex coordinates and
%  endpoints on each frame (~30 ms). Calling show() per frame would instead
%  re-create the robot's whole mesh every time (~170 ms per frame, about
%  6 frames per second), which is what makes an interactive window feel laggy.
%  show() is still used for the static "home configuration" figure (section 6),
%  and the forward kinematics still comes from the rigidBodyTree.
%
%  REQUIREMENTS
%  ------------
%   * MATLAB R2020b or newer (developed and tested on R2024b).
%   * Robotics System Toolbox  (rigidBodyTree, show, ...)
%  The symbolic HTMs are built by a small term algebra inside this file, so the
%  Symbolic Math Toolbox is not needed. That is also what makes it possible to
%  compile the same code into a standalone .exe (see the standalone folder).
%
%  QUICK START
%  -----------
%   1. Press Run (F5).
%   2. Fill in the pop-up form and press "Build".
%      (Press "Use defaults" to skip it and use the defaults in section 1.)
%   3. The console shows the DH table, every HTM, the robot summary, the
%      numeric cross-check and the planar/spatial verdict.
%   4. In the "live control" window: drag a slider (continuous animation) or
%      type a number in a box and press Enter (smooth animation). The robot,
%      the numeric HTMs and the symbolic HTMs all update.
%
%  EXAMPLE INPUTS TO TRY (type these in the form)
%  ---------------------------------------------
%   joints RRR,  lengths 0.40 0.30 0.20            -> planar 3R
%   joints RRR,  lengths 0.40 0.30 0.20, spatial   -> spatial 3R
%   joints RPR,  lengths 0.40 0.25 0.30            -> prismatic slide
%   joints RRR,  lengths 0.40 0 0.20               -> zero link length OK
%   joints RRR,  lengths 0.40 0.30 0.20, twists 0 0 0, offsets 0.10 0 0
%   joints RRPR, lengths 0.35 0.25 0.20 0, offsets 0.50 0 0 0   -> SCARA
%         (shoulder at 0.50 m, q3 = -150 mm lowers the wrist to z = 0.35 m;
%          with a 180 deg twist on joint 2 the same slide is +150 mm down and
%          the tool frame points down, as a pick-and-place head does)
%
%  ========================================================================


%% 0. Environment check
% Confirm that the Robotics System Toolbox is installed before doing work.
% (The symbolic HTMs are built by a small term algebra further down, so the
% Symbolic Math Toolbox is NOT needed - which is also what lets this code be
% compiled into a standalone .exe.)

haveRobotics = ~isempty(ver('robotics'));   % Robotics System Toolbox
assert(haveRobotics, ['This script requires the Robotics System Toolbox. ' ...
    'Install it from Add-Ons > Get Add-Ons, then run again.']);
fprintf('Toolboxes OK: Robotics System Toolbox.\n');


%% 1. Configuration - asked for in a pop-up form
% The values below are the DEFAULTS: they pre-fill the form, and they are used
% as-is if you press "Use defaults" (or close the form).
%
%   jointSeq       one letter per joint, 'R' (revolute) or 'P' (prismatic)
%   linkLengths    link length of each joint in METRES (0 is allowed; at
%                  least one must be positive)
%   jointTwist_deg optional DH twist alpha_i of each joint, in DEGREES
%                  ([] = all zero = planar arm; set one to 90 for spatial)
%   baseOffsets    optional d_i offset of each joint, in METRES

defaultJointSeq    = 'RRR';            % try 'RR', 'RP', 'RPR', 'RRR', 'RRRR'
defaultLinkLengths = [0.40 0.30 0.20]; % metres, one per joint (0 is allowed)
defaultTwist_deg   = [];               % e.g. [90 0 0] for a spatial arm
defaultBaseOffsets = [];               % e.g. [0.10 0 0] for a raised base

useInputDialog     = true;   % false -> skip the form and use the defaults

if useInputDialog
    cfg = dhInputDialog(defaultJointSeq, defaultLinkLengths, ...
                        defaultTwist_deg, defaultBaseOffsets);
else
    cfg = struct('jointSeq',defaultJointSeq, ...
                 'linkLengths',defaultLinkLengths, ...
                 'twist_deg',defaultTwist_deg, ...
                 'baseOffsets',defaultBaseOffsets);
end

% The window is built by a function, not by straight-line script code, so the
% "New robot" button can call it again with another configuration and get a
% fresh window in the SAME MATLAB session - no restart, no re-running the file.
% Its outputs are named exactly as the old top-level variables were, so the
% DH table, the symbolic model and the robot are still in the workspace.
[DH, SB, robot, S, fig] = dhLaunchApp(cfg);


%% 8. Local functions
% Everything below this line is a local function. They must stay at the very
% end of the file. The form, the slider timer and the callbacks all call them.

function [DH, SB, robot, S, fig] = dhLaunchApp(cfg)
% Build the whole live control window for one robot configuration.
% Called once by the script, and again by dhNewRobot when you define a
% different robot from inside the window.
%
% Returns the pieces a reader of this script is most likely to want in the
% workspace (and that dhNewRobot rebuilds): the DH table, the symbolic model,
% the Robotics System Toolbox chain, the live state and the window handle.

jointSeq       = upper(strtrim(char(cfg.jointSeq)));
linkLengths    = double(cfg.linkLengths(:))';
jointTwist_deg = double(cfg.twist_deg(:))';
baseOffsets    = double(cfg.baseOffsets(:))';
N              = numel(jointSeq);

% The form already checks this; re-check here for the "no form" path.
cfgErr = dhConfigError(jointSeq, linkLengths, jointTwist_deg, baseOffsets);
assert(isempty(cfgErr), 'Configuration error: %s', cfgErr);

if isempty(jointTwist_deg), jointTwist_deg = zeros(1,N); end
if isempty(baseOffsets),    baseOffsets    = zeros(1,N); end
jointTwist_rad = jointTwist_deg * pi/180;

% Planar means every joint axis is parallel, i.e. every twist is a multiple
% of 180 degrees (sin(alpha) == 0).
isPlanar = all(abs(sin(jointTwist_rad)) < 1e-9);
if isPlanar
    armType = 'PLANAR';  typeStr = '(planar)';
else
    armType = 'SPATIAL'; typeStr = '(spatial)';
end
fprintf('Robot: %d DOF, joints = %s, lengths = [%s] m.\n', ...
    N, jointSeq, strtrim(sprintf('%.3f ', linkLengths)));
fprintf('       twists = [%s] deg -> %s robot.\n', ...
    strtrim(sprintf('%g ', jointTwist_deg)), armType);
if isPlanar
    fprintf(['  Tip: give a joint a 90 deg twist (or tick "spatial" in ' ...
             'the form) to make the arm leave the plane.\n']);
end


%% 2. Generate the DH parameters from the joint sequence
%   * Revolute  ('R'): the moving variable is theta_i; a_i takes the link
%                      length; d_i is the structural offset.
%   * Prismatic ('P'): the moving variable is d_i; a_i = 0 and the link
%                      length defines the stroke of the slide.

DH = dhGenerate(jointSeq, linkLengths, jointTwist_rad, baseOffsets);

dhTable = table((1:N)', cellstr(jointSeq(:)), DH.alpha(:), DH.a(:), ...
    DH.d(:), DH.theta(:), DH.variable(:), ...
    'VariableNames', {'Joint','Type','alpha_rad','a_m','d_m', ...
                      'theta_rad','MovingVar'});
disp(dhTable);


%% 3. Step-by-step SYMBOLIC Homogeneous Transformation Matrices (general form)
% Each joint contributes one standard DH matrix:
%
% $$T_i^{\,i-1}=\begin{bmatrix}\cos\theta_i & -\sin\theta_i\cos\alpha_i & \sin\theta_i\sin\alpha_i & a_i\cos\theta_i\\ \sin\theta_i & \cos\theta_i\cos\alpha_i & -\cos\theta_i\sin\alpha_i & a_i\sin\theta_i\\ 0 & \sin\alpha_i & \cos\alpha_i & d_i\\ 0 & 0 & 0 & 1\end{bmatrix}$$
%
% The matrices are kept in GENERAL form: the link lengths appear as a1, a2...,
% the joint variables as theta1, theta2... (or d1, d2... for prismatic
% joints), and a twist/offset appears as alpha_i / d_i only when it is
% actually needed (i.e. when it is not zero). So for a 3-link planar arm you
% get exactly the classic a1, a2, a3 and theta1, theta2, theta3.
%
% Angle SUMS are written with the conventional kinematics shorthand
%
%     C1   = cos(theta1)                     S1   = sin(theta1)
%     C12  = cos(theta1 + theta2)            S12  = sin(theta1 + theta2)
%     C123 = cos(theta1 + theta2 + theta3)   S123 = sin(theta1 + theta2 + theta3)
%
% (and so on, for any sum of joint angles; the sums may skip a prismatic
% joint, so an RPR arm shows C13 = cos(theta1 + theta3)). That is what keeps
% the composed matrix short enough to read. The script prints the legend, and
% the live window repeats it above the matrices. The shorthand is display
% only: the same expressions are kept as coefficient/angle-sum terms, and
% section 5 evaluates those terms for the numeric cross-check.

SB = dhSymbolic(DH);          % general matrices + the composed one

if ~isempty(SB.abbrev)
    legLines = dhAbbrevLines(SB.abbrev, 110, 'Shorthand:  ');
    fprintf('\n');
    fprintf('%s\n', legLines{:});
end

for i = 1:N
    fprintf('\n--- T(%d -> %d)   [joint %d, %s] ---\n', ...
        i-1, i, i, DH.variable{i});
    rows = dhSymRows(SB.TgenAbbr{i});
    fprintf('%s\n', rows{:});
end
fprintf('\n--- End-effector transform T(0 -> %d)   (composed, simplified) ---\n', N);
rowsTot = dhSymRows(SB.TtotAbbr);
fprintf('%s\n', rowsTot{:});


%% 4. Build the kinematic chain with the Robotics System Toolbox
% One rigid body per joint, joint type from the letter, DH row applied with
% setFixedTransform(...,'dh'), and physical visual geometry added:
%   * a cylinder spanning the link (its length is the link length),
%   * a sphere at each joint,
%   * a rail on each body whose next joint is prismatic.

robot = dhBuildRobot(DH, linkLengths);
showdetails(robot)


%% 5. Verify that the two descriptions agree
% Evaluate the general matrix at the test configuration (and the numeric link
% lengths) and compare it with the toolbox forward kinematics.

qTest = rand(1,N) - 0.5;                       % an arbitrary configuration
T_sym = dhEvalTerms(SB.TtotTerms, qTest, linkLengths, DH.d, DH.isRev);
T_rbt = getTransform(robot, qTest(:), sprintf('link%d',N));
fprintf('Test configuration q = [%s]\n', strtrim(sprintf('%.3f  ', qTest)));
fprintf('Max |T_symbolic - T_toolbox| = %.3e\n', max(abs(T_sym(:)-T_rbt(:))));


%% 6. Plot the robot at its home configuration
% show() draws the X, Y and Z coordinate frame of every joint; the frames are
% redrawn thicker, and dashed projection lines show the x, y and z components
% of the end position.

figHome = figure('Name','Robot - home configuration','Color','w');
axHome  = axes(figHome);
show(robot, zeros(N,1), 'Parent', axHome, 'Frames','on', 'PreservePlot',false);
applyView(axHome, sum(linkLengths), ...
    dhPlotZRange(robot, DH, linkLengths, isPlanar, sum(linkLengths)));
hold(axHome,'on');
TframesH    = cell(1,N+1);
TframesH{1} = eye(4);
for i = 1:N
    TframesH{i+1} = getTransform(robot, zeros(N,1), sprintf('link%d',i));
end
dhDrawFrames(axHome, TframesH, sum(linkLengths));
pEndHome = TframesH{N+1}(1:3,4);
dhDrawProjections(axHome, pEndHome, sum(linkLengths));
hold(axHome,'off');
dhTidyPlot(figHome);
title(axHome, sprintf('Home configuration - %d-DOF %s %s', ...
    N, jointSeq, typeStr));


%% 7. Live control window
% Each joint and each link gets a row:  [ label ] [ slider ] [ text box ].
%
%   * REVOLUTE joints  : DEGREES  (-180 .. 180)
%   * PRISMATIC joints : MILLIMETRES (-stroke .. +stroke; it travels both ways)
%   * LINK LENGTHS     : MILLIMETRES (0 .. 1000; zero is allowed)
%     (for a prismatic joint the row is the STROKE s_i, i.e. the travel each
%      way, and the DH link length a_i stays 0)
%
% Below the plot a scrollable pane shows, for every joint and for the whole
% chain, the SYMBOLIC general matrix next to its NUMERIC value (2 d.p.), with
% the angle sums in the usual C12 / S123 shorthand (the legend sits on top):
%
%     [ C1*a1, ... ]      |   [   0.33, ... ]
%
% plus the joint values, the end position and every frame origin.

% The symbolic blocks depend on WHICH link lengths are zero - a zero a_i is
% written as 0, not as 'a1*cos(...)' - but not on the joint values. dhRebuild
% rebuilds them whenever a link is slid through zero (see its comment).
symBlock = dhSymBlock(SB, DH);
symTitles   = symBlock.titles;
symRowsMx   = symBlock.rows;
abbrevLines = symBlock.abbrevLines;
symW        = symBlock.symW;

% Size the window to the screen. MATLAB scales an oversized figure down when
% it runs in the desktop, but the COMPILED application keeps the requested size
% - and then the top of the window (the plot title and the view buttons) sits
% above the screen edge and cannot be reached.
scr  = get(0,'ScreenSize');                       % [1 1 width height]
winW = min(1340, scr(3) - 60);
winH = min(1000, scr(4) - 90);
winL = max(10, min(30, scr(3) - winW - 10));
winB = max(20, scr(4) - winH - 40);
fig = figure('Name','DH robot - live control','NumberTitle','off', ...
    'Color',[0.94 0.94 0.94],'MenuBar','none','ToolBar','figure', ...
    'Position',[winL winB winW winH]);

% The axes sits in its own band. Its bottom edge is kept clear of the readout
% (and the slider column is kept left of it) so that the axis labels, the tick
% labels and anything you zoom into stay inside the plot band and never reach
% the controls. The band is deliberately TALL: for a planar arm MATLAB scales
% the 3-D box to the shorter side of the axes rectangle, so height - not width
% - is what decides how big the arm is drawn. The band therefore runs from just
% above the readout up to the top of the window (the title needs a little room
% above the position rectangle), and the slider column, which ends higher up,
% does not compete for that space.
ax = axes(fig,'Units','normalized', ...
    'Position',[0.28 0.455 0.640 0.517],'Box','on');
try, ax.Toolbar.Visible = 'off'; catch, end   % else its toolbar covers the plot

% Scrollable readout along the bottom. Max = 2 makes it a multi-selection
% list box, which is what allows an empty Value (so no row is highlighted).
% At the default window size it shows ~163 monospaced characters per line, so
% dhRedraw keeps every line at 158 characters or fewer (no scroll bar).
readout = uicontrol(fig,'Style','listbox','Units','normalized', ...
    'Position',[0.008 0.012 0.978 0.42],'BackgroundColor','w', ...
    'FontName','FixedWidth','FontSize',10,'Min',0,'Max',2,'String',{});

% --- view buttons: XY, YZ, ZX and back to 3-D ----------------------------
% Real controls in the free strip under the sliders (above the readout).
%
% They are NOT in the plot area, deliberately: the compiled application paints
% the 3-D axes on a canvas that covers everything over it, and the figure is
% wider than the screen there, so a button in the corner of the plot is either
% invisible or off-screen. Text drawn inside the axes has the opposite problem:
% it is anchored in the data box, so it slides away as soon as the camera turns
% - which left no way back once a plane view had been chosen.
%
% Here they are always on screen, never move when the camera turns, and the
% button of the view that is on screen is highlighted.
vbW   = 0.060;
vbH   = 0.034;
vbGap = 0.005;
vbY   = 0.447;
viewNames   = {'XY','YZ','ZX','3D'};
viewTips    = {'Look straight down the XY plane (plan view)', ...
               'Look straight at the YZ plane', ...
               'Look straight at the ZX plane', ...
               'Back to the 3-D perspective view'};
for k = 1:numel(viewNames)
    uicontrol(fig,'Style','pushbutton','Units','normalized', ...
        'Position',[0.008 + (k-1)*(vbW+vbGap), vbY, vbW, vbH], ...
        'String',viewNames{k},'FontSize',9,'FontWeight','bold', ...
        'TooltipString',viewTips{k},'Tag','viewbutton', ...
        'Callback', @(~,~) dhSetPlaneView(fig, viewNames{k}));
end

% --- "New robot": another arm in the same session -------------------------
% The whole window is built by dhLaunchApp, so this button only has to ask for
% a new definition and call it again. The form opens pre-filled with what is on
% screen, so a small change (say, a fourth joint) is one edit instead of a
% retype - and nothing has to be closed and restarted by hand.
uicontrol(fig,'Style','pushbutton','Units','normalized', ...
    'Position',[0.008 + numel(viewNames)*(vbW+vbGap) + 0.012, vbY, 0.090, vbH], ...
    'String','New robot','FontSize',9,'FontWeight','bold', ...
    'TooltipString',['Define a different robot (joint sequence, lengths, ' ...
                     'twists, offsets) without restarting the program'], ...
    'Callback', @(~,~) dhNewRobot(fig));

% --- geometry of the slider column (it stops above the readout) ----------
colTop  = 0.985;                 % top of the first row
colBot  = 0.50;                  % bottom limit (kept above the readout)
nRows   = 2*N + 2;               % 2 header rows + N joints + N lengths
rowH    = min((colTop - colBot)/nRows, 0.07);
topY    = colTop;

% column geometry for the three parts of a row
xLab = 0.008; wLab = 0.075;
xSld = 0.088; wSld = 0.105;
xEdt = 0.198; wEdt = 0.075;
slH  = min(0.7*rowH, 26/1000);   % cap the slider height so it stays a slider
edH  = min(0.8*rowH, 26/1000);

% --- header: joint variables --------------------------------------------
uicontrol(fig,'Style','text','Units','normalized', ...
    'Position',[xLab topY-rowH 0.26 rowH], ...
    'String','Joint variables  (deg / mm)','FontWeight','bold','FontSize',11, ...
    'HorizontalAlignment','left');

jointSliders = gobjects(1,N);    % handle arrays used by the callbacks
jointEdit    = gobjects(1,N);
for i = 1:N
    y = topY - (i+1)*rowH;
    if DH.isRev(i)
        tTxt = sprintf('q_%d (R, deg)',i);
        lims = [-180 180];               % degrees
        init = 0;
    else
        % A slide travels BOTH ways from its zero: the stroke is the travel
        % EACH way, so a SCARA lift can go down (-) as well as up (+), and a
        % cylindrical column can rise from its base. The old 0..stroke range
        % made "down" impossible.
        tTxt = sprintf('q_%d (P, mm)',i);
        st   = max(linkLengths(i)*1000, 0.001);       % millimetres
        lims = [-st st];
        init = 0;
    end
    uicontrol(fig,'Style','text','Units','normalized', ...
        'Position',[xLab y wLab rowH],'String',tTxt,'FontSize',10, ...
        'HorizontalAlignment','left');
    sl = uicontrol(fig,'Style','slider','Units','normalized', ...
        'Position',[xSld y+(rowH-slH)/2 wSld slH], ...
        'Min',lims(1),'Max',lims(2),'Value',init);
    ed = uicontrol(fig,'Style','edit','Units','normalized', ...
        'Position',[xEdt y+(rowH-edH)/2 wEdt edH], ...
        'String',sprintf('%.1f',init),'BackgroundColor','w','FontSize',10);
    jointSliders(i) = sl;
    jointEdit(i)    = ed;
end

% --- header: link lengths -----------------------------------------------
uicontrol(fig,'Style','text','Units','normalized', ...
    'Position',[xLab topY-(N+2)*rowH 0.26 rowH], ...
    'String','Link lengths  (mm)','FontWeight','bold','FontSize',11, ...
    'HorizontalAlignment','left');

lengthSliders = gobjects(1,N);
lengthEdit    = gobjects(1,N);
for i = 1:N
    y = topY - (N+2+i)*rowH;
    if DH.isRev(i)
        lenTxt = sprintf('a_%d (mm)',i);       % link length of a revolute joint
    else
        lenTxt = sprintf('s_%d (mm)',i);       % STROKE of a slide, not a link
    end
    uicontrol(fig,'Style','text','Units','normalized', ...
        'Position',[xLab y wLab rowH],'String',lenTxt, ...
        'FontSize',10,'HorizontalAlignment','left');
    sl = uicontrol(fig,'Style','slider','Units','normalized', ...
        'Position',[xSld y+(rowH-slH)/2 wSld slH], ...
        'Min',0,'Max',1000,'Value',linkLengths(i)*1000);   % 0 is allowed
    ed = uicontrol(fig,'Style','edit','Units','normalized', ...
        'Position',[xEdt y+(rowH-edH)/2 wEdt edH], ...
        'String',sprintf('%.0f',linkLengths(i)*1000), ...
        'BackgroundColor','w','FontSize',10);
    lengthSliders(i) = sl;
    lengthEdit(i)    = ed;
end

% --- the graphics scene -------------------------------------------------
% Every drawing object is created ONCE, right here, and the animation only
% rewrites coordinates afterwards (see dhCreateScene / dhUpdateVisuals).
% hold on is essential: without it the first plot3 call would wipe the scene,
% because a fresh axes uses NextPlot = 'replace'.
hold(ax,'on');
set(ax,'NextPlot','add');
view(ax,3);
G = dhCreateScene(ax, N);
camlight(ax,'headlight');
lighting(ax,'gouraud');
material(ax,'dull');
xlabel(ax,'X'); ylabel(ax,'Y'); zlabel(ax,'Z');

% --- shared state, callbacks and the continuous-animation timer ---------
S = struct('jointSeq',jointSeq,'jointAlpha',DH.alpha, ...
           'baseOffsets',DH.d,'N',N,'typeStr',typeStr,'isRev',DH.isRev, ...
           'isPlanar',isPlanar, ...
           'jointSliders',jointSliders,'jointEdit',jointEdit, ...
           'lengthSliders',lengthSliders,'lengthEdit',lengthEdit, ...
           'ax',ax,'readout',readout,'robot',robot,'DH',DH,'g',G, ...
           'symTitles',{symTitles},'symRows',{symRowsMx},'symW',symW, ...
           'abbrevLines',{abbrevLines}, ...
           'lastJoint',zeros(1,N),'lastLen',linkLengths*1000, ...
           'animating',false,'tLastRebuild',tic,'tLastReadout',tic, ...
           'textStale',false,'planeView','');
setappdata(fig,'dhstate',S);

for i = 1:N
    set(jointSliders(i), 'Callback', @(s,e) dhPoll(fig, true));
    set(lengthSliders(i),'Callback', @(s,e) dhLengthRelease(fig));
    set(jointEdit(i),    'Callback', @(s,e) dhJointEdit(fig,i));
    set(lengthEdit(i),   'Callback', @(s,e) dhLengthEdit(fig,i));
end

% The timer watches the sliders while you drag and animates continuously.
S.timer = timer('ExecutionMode','fixedSpacing','Period',0.04, ...
    'BusyMode','drop','TimerFcn', @(~,~) dhPoll(fig, false));
setappdata(fig,'dhstate',S);
dhMarkViewButton(fig, '');          % the window opens in the 3-D view
set(fig,'CloseRequestFcn', @(~,~) dhClose(fig));

dhRebuild(fig);          % first draw for the initial configuration
dhSyncEdits(fig);
start(S.timer);

fprintf('\nLive control window is open. Drag a slider or type a value.\n');

% In the compiled application the Command Window does not exist, so the same
% report goes to a window of its own. It is called from HERE (not from the top
% level) so that a robot defined with the "New robot" button reports as well.
if isdeployed
    dhDeployedReport(jointSeq, linkLengths, jointTwist_deg, N, armType, ...
        typeStr, DH, SB, T_sym, T_rbt, robot);
end
end


% ------------------------------------------------------------------ the form
function cfg = dhInputDialog(dSeq, dLen, dTwist, dOff)
% Modal pop-up form that asks for the robot definition. Returns a struct with
% fields jointSeq (char), linkLengths (numeric), twist_deg (numeric),
% baseOffsets (numeric). Closing the form keeps the defaults.
col = [0.96 0.96 0.96];
f = figure('Name','Robot configuration - DH generator', ...
    'NumberTitle','off','MenuBar','none','ToolBar','none','Resize','off', ...
    'WindowStyle','modal','Position',[480 340 680 380],'Color',col);

uicontrol(f,'Style','text','Position',[22 336 640 26], ...
    'String','Define the robot, then press "Build".','FontSize',12, ...
    'FontWeight','bold','HorizontalAlignment','left','BackgroundColor',col);

h.seq   = dhFormRow(f, 282, 'fseq', ...
    'Joint sequence   (R = revolute,  P = prismatic;  2 to 4 letters):', ...
    dSeq, col);
h.len   = dhFormRow(f, 228, 'flen', ...
    'Link lengths in METRES, one per joint (0 is allowed):', ...
    strtrim(sprintf('%.3f ', dLen)), col);
h.twist = dhFormRow(f, 174, 'ftwist', ...
    'Joint twists alpha in DEGREES, optional (e.g. 90 0 0):', ...
    dhNumList(dTwist), col);
h.off   = dhFormRow(f, 120, 'foff', ...
    'Base offsets d in METRES, optional (e.g. 0.10 0 0):', ...
    dhNumList(dOff), col);

h.spatial = uicontrol(f,'Style','checkbox','Position',[24 86 620 22], ...
    'String','Make it spatial  (twist joint 1 by 90 degrees)', ...
    'Value',0,'BackgroundColor',col,'FontSize',10,'Tag','fspatial');

h.msg = uicontrol(f,'Style','text','Position',[24 54 620 22],'String','', ...
    'HorizontalAlignment','left','BackgroundColor',col, ...
    'ForegroundColor',[0.80 0 0],'FontSize',10,'Tag','fmsg');

uicontrol(f,'Style','pushbutton','Position',[444 14 100 30], ...
    'String','Build','FontSize',10,'FontWeight','bold', ...
    'Callback',@(~,~) dhFormBuild(f));
uicontrol(f,'Style','pushbutton','Position',[554 14 110 30], ...
    'String','Use defaults','FontSize',10, ...
    'Callback',@(~,~) uiresume(f));

% The result lives on the root so it survives the figure being closed. It
% starts as the defaults, so closing the form simply keeps them.
setappdata(0, 'dhFormResult', struct('jointSeq',dSeq, ...
    'linkLengths',dLen(:)','twist_deg',dTwist(:)','baseOffsets',dOff(:)'));

uiwait(f);
cfg = getappdata(0,'dhFormResult');
rmappdata(0,'dhFormResult');
if isgraphics(f), delete(f); end
end


function h = dhFormRow(f, y, tag, label, value, col)
% One labelled edit box of the form; the edit box is returned.
uicontrol(f,'Style','text','Position',[24 y+26 620 20],'String',label, ...
    'HorizontalAlignment','left','BackgroundColor',col,'FontSize',10);
h = uicontrol(f,'Style','edit','Position',[24 y 620 26],'String',value, ...
    'HorizontalAlignment','left','BackgroundColor','w','FontSize',10, ...
    'Tag',tag);
end


function dhFormBuild(f)
% Validate the form and, if it is good, store the result and close the form.
seq = upper(strtrim(get(findobj(f,'Tag','fseq'),'String')));
seq = seq(~isspace(seq));
L   = dhParseList(get(findobj(f,'Tag','flen'),'String'));
tw  = dhParseList(get(findobj(f,'Tag','ftwist'),'String'));
of  = dhParseList(get(findobj(f,'Tag','foff'),'String'));

if get(findobj(f,'Tag','fspatial'),'Value') == 1
    if isempty(tw), tw = zeros(1,numel(seq)); end
    if numel(tw) == numel(seq), tw(1) = 90; end
end

err = dhConfigError(seq, L, tw, of);
if ~isempty(err)
    set(findobj(f,'Tag','fmsg'),'String',err);
    return;
end
setappdata(0,'dhFormResult', struct('jointSeq',seq,'linkLengths',L, ...
    'twist_deg',tw,'baseOffsets',of));
uiresume(f);
end


function err = dhConfigError(seq, L, tw, of)
% Shared validation of a robot definition; '' means the definition is fine.
err = '';
if isempty(seq) || ~all(ismember(seq,'RP'))
    err = 'Joint sequence may contain only R and P (e.g. RRR or RPR).';
    return;
end
N = numel(seq);
if N < 2 || N > 4
    err = sprintf('Use 2 to 4 joints (you gave %d).', N);
    return;
end
if numel(L) ~= N
    err = sprintf('Enter exactly %d link lengths, in metres.', N);
    return;
end
if any(L < 0)
    err = 'Link lengths must be zero or positive.';
    return;
end
if ~any(L > 0)
    err = 'At least one link length must be greater than zero.';
    return;
end
if ~isempty(tw) && numel(tw) ~= N
    err = sprintf('Twists: enter %d numbers, or leave the box empty.', N);
    return;
end
if ~isempty(of) && numel(of) ~= N
    err = sprintf('Base offsets: enter %d numbers, or leave the box empty.', N);
    return;
end
% For a slide, d_i IS the joint variable - there is no room for a separate
% fixed offset, and the toolbox drops one silently. Catch it here instead.
if ~isempty(of)
    badP = find(seq == 'P' & of(:)' ~= 0, 1);
    if ~isempty(badP)
        err = sprintf(['Base offset on joint %d is not used: for a prismatic ' ...
            'joint d_i is the sliding variable itself, and it travels both ' ...
            'ways (the stroke slider covers it).'], badP);
        return;
    end
end
end


function s = dhNumList(v)
% Render a numeric row as text for an edit box ('' for empty).
if isempty(v), s = ''; else, s = strtrim(sprintf('%g ', v(:)')); end
end


function v = dhParseList(str)
% Parse a user-typed list of numbers: spaces and/or commas separate them.
v = sscanf(strrep(strtrim(char(str)), ',', ' '), '%f')';
end


% ------------------------------------------------------------------- the math
function DH = dhGenerate(jointSeq, linkLengths, twistRad, baseOffsets)
% Generate the initial DH parameter set from the joint sequence.
jointSeq    = upper(strtrim(char(jointSeq)));
N           = numel(jointSeq);
linkLengths = double(linkLengths(:))';
if isempty(twistRad) || numel(twistRad) ~= N, twistRad = zeros(1,N); end
if isempty(baseOffsets) || numel(baseOffsets) ~= N, baseOffsets = zeros(1,N); end

isRev = false(1,N);
for i = 1:N
    isRev(i) = (jointSeq(i) == 'R');
end

DH.N        = N;
DH.isRev    = isRev;
DH.alpha    = double(twistRad(:))';
DH.d        = double(baseOffsets(:))';
DH.theta    = zeros(1,N);
DH.a        = zeros(1,N);
DH.variable = cell(1,N);
for i = 1:N
    if isRev(i)
        DH.a(i)        = linkLengths(i);      % revolute: a_i is the link
        DH.variable{i} = sprintf('theta%d',i);
    else
        DH.a(i)        = 0;                   % prismatic: no a offset
        DH.variable{i} = sprintf('d%d',i);
    end
end
end


function SB = dhSymbolic(DH)
% Build the general (symbolic) form of every HTM - WITHOUT the Symbolic Math
% Toolbox, so the same code also runs inside a compiled .exe (the Symbolic Math
% Toolbox cannot be deployed).
%
% The matrices are needed twice:
%   * as TEXT, in the general form the lesson is about (a_i, theta_i, d_i and
%     alpha_i where they are actually non-zero), with the usual C1 / S12
%     shorthand for angle sums;
%   * as something that can be EVALUATED, for the numeric cross-check against
%     the toolbox forward kinematics in section 5.
%
% A DH matrix entry only ever contains cos/sin of an angle sum, times a number,
% times at most one link length or offset. Products of two such entries stay in
% the same family, using the product-to-sum identities
%
%   cos u cos v = (cos(u+v) + cos(u-v)) / 2   sin u sin v = (cos(u-v) - cos(u+v)) / 2
%   sin u cos v = (sin(u+v) + sin(u-v)) / 2   cos u sin v = (sin(u+v) - sin(u-v)) / 2
%
% so T(0 -> N) is a sum of terms
%
%   coefficient * cos/sin(sum of joint angles) * ... * (a_i and/or d_i)
%
% which is exactly the classic  C123,  C1*C23,  C1*a1 + C12*a2 + C123*a3  form.
% The little algebra below (dhMulOne / dhAddTerms) applies those rules, so no
% computer-algebra system is needed and the arithmetic is exact.
%
% SB.Tgen{i}      the general matrix of joint i, as a 4x4 cell of text
% SB.TgenTerms{i} the same matrix as terms (for evaluating it)
% SB.TtotTerms    the composed T(0 -> N) as terms
% SB.TtotAbbr     the composed T(0 -> N) as text
% SB.TgenAbbr     = SB.Tgen (a single joint is already in abbreviated form)
% SB.abbrev       the abbreviations that occur, in display order
N = DH.N;

% Two joints may only be added into one angle sum (C1*C2 -> C12) when their
% axes are parallel, i.e. when every twist between them is 0 or 180 deg. Across
% a 90 deg twist the product is kept as a product (C1*C23), as in the books.
planarTwist = abs(sin(DH.alpha)) < 1e-9;

SB.Tgen      = cell(1,N);
SB.TgenTerms = cell(1,N);
for i = 1:N
    [SB.Tgen{i}, SB.TgenTerms{i}] = dhJointGeneral(DH, i);
end

Tt = SB.TgenTerms{1};
for i = 2:N
    Tt = dhMatMul(Tt, SB.TgenTerms{i}, planarTwist);
end
SB.TtotTerms = Tt;

SB.TgenAbbr = SB.Tgen;
SB.TtotAbbr = dhTermsMatText(Tt);

% which abbreviations actually occur? (C1, S12, S123, ...)
used = {};
for i = 1:N
    used = [used, dhAtomsIn(SB.TgenTerms{i})];       %#ok<AGROW>
end
used = unique([used, dhAtomsIn(Tt)]);
if isempty(used)
    SB.abbrev = {};
else
    % order: C before S, short angle sums before long ones
    keys = cellfun(@(s) sprintf('%s%02d%s', s(1), numel(s)-1, s), ...
        used, 'UniformOutput', false);
    [~, ord] = sort(keys);
    SB.abbrev = used(ord);
end
end


function [M, T] = dhJointGeneral(DH, i)
% The standard DH matrix of joint i in GENERAL form.
%
% M (4x4 text) is what the display shows: the moving variable, the link length
% a_i and - only where they are non-zero - the twist alpha_i and the offset d_i
% stay as symbols. T (4x4 terms) is the same matrix with the twist taken
% numerically and a_i/d_i as symbols, ready to be multiplied into T(0 -> N).
N   = DH.N;
rev = DH.isRev(i);
ze  = zeros(1,N);
v   = ze;  v(i) = double(rev);            % this joint's angle unit vector

if DH.a(i) ~= 0, aTxt = sprintf('a%d',i); else, aTxt = '0'; end
if rev
    if DH.d(i) ~= 0, dTxt = sprintf('d%d',i); else, dTxt = '0'; end
else
    dTxt = sprintf('d%d',i);              % prismatic: d_i is the moving variable
end
if rev
    cTxt = sprintf('C%d',i);   sTxt = sprintf('S%d',i);
else
    cTxt = '1';                sTxt = '0'; % its theta is fixed at 0
end
if DH.alpha(i) ~= 0
    caTxt = sprintf('cos(alpha%d)',i);   saTxt = sprintf('sin(alpha%d)',i);
else
    caTxt = '1';                         saTxt = '0';
end

M = repmat({'0'},4,4);
M{1,1} = cTxt;
M{1,2} = dhMulTxt({'-',sTxt,caTxt});
M{1,3} = dhMulTxt({sTxt,saTxt});
M{1,4} = dhMulTxt({aTxt,cTxt});
M{2,1} = sTxt;
M{2,2} = dhMulTxt({cTxt,caTxt});
M{2,3} = dhMulTxt({'-',cTxt,saTxt});
M{2,4} = dhMulTxt({aTxt,sTxt});
M{3,2} = saTxt;
M{3,3} = caTxt;
M{3,4} = dTxt;
M{4,4} = '1';

ca = cos(DH.alpha(i));
sa = sin(DH.alpha(i));
T  = repmat({dhT0()},4,4);
if rev
    T{1,1} = dhAtomTerm(1,   1, v, {});
    T{1,2} = dhAtomTerm(-ca, 2, v, {});
    T{1,3} = dhAtomTerm(sa,  2, v, {});
    T{2,1} = dhAtomTerm(1,   2, v, {});
    T{2,2} = dhAtomTerm(ca,  1, v, {});
    T{2,3} = dhAtomTerm(-sa, 1, v, {});
    T{3,2} = dhNumTerm(sa, {});
    T{3,3} = dhNumTerm(ca, {});
else
    % A prismatic joint has theta fixed at 0, so the theta-dependent entries
    % collapse - but its twist alpha_i does NOT. The rotation of a slide is
    % exactly [[1 0 0]; [0 ca -sa]; [0 sa ca]], and dropping that here used to
    % make the composed matrix of any RPP/RPR arm with a twisted slide come out
    % planar, while the numeric matrix (from the robot) still had the twist.
    T{1,1} = dhNumTerm(1,   {});
    T{2,2} = dhNumTerm(ca,  {});
    T{2,3} = dhNumTerm(-sa, {});
    T{3,2} = dhNumTerm(sa,  {});
    T{3,3} = dhNumTerm(ca,  {});
end
if DH.a(i) ~= 0
    sy = {sprintf('a%d',i)};
    T{1,4} = dhAtomTerm(1, 1, v, sy);
    T{2,4} = dhAtomTerm(1, 2, v, sy);
end
if rev
    if DH.d(i) ~= 0
        T{3,4} = dhNumTerm(1, {sprintf('d%d',i)});
    end
else
    T{3,4} = dhNumTerm(1, {sprintf('d%d',i)});
end
T{4,4} = dhNumTerm(1, {});
end


function s = dhMulTxt(f)
% Multiply text factors: drop '1', collapse to '0', remember a leading '-'.
% dhMulTxt({'0','C1'}) -> '0'   dhMulTxt({'-','S1','cos(alpha1)'}) -> '-S1*cos(alpha1)'
neg = false;
parts = {};
for k = 1:numel(f)
    x = f{k};
    if strcmp(x,'-')
        neg = ~neg;
    elseif strcmp(x,'0')
        s = '0';
        return
    elseif ~strcmp(x,'1')
        parts{end+1} = x;                              %#ok<AGROW>
    end
end
if isempty(parts)
    s = '1';
else
    s = strjoin(parts,'*');
end
if neg
    s = ['-' s];
end
end


% ------------------------------------------------- the little term algebra --
% A term is
%     c * cos/sin(sum1) * cos/sin(sum2) * ... * symbol1 * symbol2 * ...
% with
%   c   numeric coefficient
%   t   one type per atom: 1 = cos, 2 = sin (a term with no atoms is a number)
%   v   one row per atom: how much of each joint angle is in that sum
%       (always 0 for prismatic joints, whose theta is fixed at 0)
%   s   cell of symbol names multiplied in, e.g. {'a1'} or {'d2'}
% A sum of terms is a struct array of these; the empty one is dhT0().

function A = dhT0()
A = struct('c',{},'t',{},'v',{},'s',{});
end


function A = dhAtomTerm(c, tt, vv, sy)
% One term made of a single cos/sin atom.
if nargin < 4, sy = {}; end
A = dhCanon(struct('c',c,'t',tt,'v',double(vv(:)'),'s',{sy}));
end


function A = dhNumTerm(c, sy)
% One term without trig atoms: a number times symbols.
if nargin < 2, sy = {}; end
A = dhCanon(struct('c',c,'t',zeros(1,0),'v',zeros(0,0),'s',{sy}));
end


function T = dhCanon(T)
% Drop what is zero, put every atom in a canonical form and order the atoms.
%   cos(-a) =  cos(a)      sin(-a) = -sin(a)      sin(0) = 0      cos(0) = 1
if isempty(T), T = dhT0(); return; end
if abs(T.c) < 1e-12, T = dhT0(); return; end
if abs(abs(T.c) - 1) < 1e-12, T.c = sign(T.c); end   % cos(90 deg) comes out 6e-17
for i = 1:numel(T.t)
    v  = T.v(i,:);
    nz = find(v ~= 0, 1);
    if ~isempty(nz) && v(nz) < 0
        v = -v;
        if T.t(i) == 2, T.c = -T.c; end
    end
    if T.t(i) == 2 && ~any(v)
        T = dhT0();
        return
    end
    if T.t(i) == 1 && ~any(v)
        T.t(i) = 0;                      % cos(0) = 1: the atom disappears
    end
    T.v(i,:) = v;
end
keep = (T.t ~= 0);
T.t  = T.t(keep);
T.v  = T.v(keep,:);
if numel(T.t) > 1
    keys = cell(1,numel(T.t));
    for i = 1:numel(T.t)
        keys{i} = dhAtomSortKey(T.t(i), T.v(i,:));
    end
    [~, ord] = sort(keys);
    T.t = T.t(ord);
    T.v = T.v(ord,:);
end
end


function k = dhAtomSortKey(t, v)
% Short sums first (C1 before C12), then by joint index, then cos before sin.
idx = find(v);
k = sprintf('%02d|%s|%d', numel(idx), sprintf('%03d', idx), t);
end


function k = dhAtomKey(t, v)
k = sprintf('%d:%s', t, sprintf('%d,', v));
end


function k = dhTermKey(T)
parts = cell(1,numel(T.t));
for i = 1:numel(T.t)
    parts{i} = dhAtomKey(T.t(i), T.v(i,:));
end
k = [strjoin(parts,'|') '#' strjoin(T.s,',')];
end


function A = dhCollect(A)
% Add up like terms of a term list.
if isempty(A), A = dhT0(); return; end
if numel(A) == 1, A = dhCanon(A); return; end
keys = cell(1,numel(A));
for k = 1:numel(A)
    keys{k} = dhTermKey(A(k));
end
[~, ~, ic] = unique(keys);
out = dhT0();
for m = 1:max(ic)
    sel = (ic == m);
    c   = sum([A(sel).c]);
    if abs(c) > 1e-9
        t1  = A(find(sel,1));
        t1.c = c;
        out(end+1) = t1;                           %#ok<AGROW>
    end
end
A = out;
end


function A = dhAddTerms(A, B)
if isempty(B), return; end
if isempty(A), A = dhCollect(B); return; end
A = dhCollect([A(:).', B(:).']);
end


function ok = dhMergeable(v1, v2, planarTwist)
% May these two angle sums be added into one? Only if all the joints between
% them rotate about parallel axes (no 90 deg twist in between).
supp = find(v1 | v2);
if isempty(supp)
    ok = true;
    return
end
lo = min(supp);
hi = max(supp);
ok = all(planarTwist(lo:hi-1));
end


function [t, v, c] = dhProductToSum(t1, v1, t2, v2)
% cos/sin product -> two atoms of the sum/difference angles, each with 1/2.
%   cos u cos v = (cos(u+v) + cos(u-v))/2      sin u sin v = (cos(u-v) - cos(u+v))/2
%   sin u cos v = (sin(u+v) + sin(u-v))/2      cos u sin v = (sin(u+v) - sin(u-v))/2
if t1 == 1 && t2 == 1
    t = [1 1];  v = [v1+v2; v1-v2];  c = [ 0.5  0.5];
elseif t1 == 2 && t2 == 2
    t = [1 1];  v = [v1-v2; v1+v2];  c = [ 0.5 -0.5];
elseif t1 == 2 && t2 == 1
    t = [2 2];  v = [v1+v2; v1-v2];  c = [ 0.5  0.5];
else
    t = [2 2];  v = [v1+v2; v1-v2];  c = [ 0.5 -0.5];
end
end


function out = dhMulOne(p, q, planarTwist)
% Product of two terms: multiply the numbers and the symbols, then attach every
% atom of q to p - merging it with a compatible atom (product-to-sum) or, across
% a twist, keeping it as another factor.
if isempty(p) || isempty(q)
    out = dhT0();
    return
end
out = p;
for k = 1:numel(out)
    out(k).c = out(k).c * q.c;
    out(k).s = [out(k).s, q.s];
end
for j = 1:numel(q.t)
    tj = q.t(j);
    vj = q.v(j,:);
    newout = dhT0();
    for k = 1:numel(out)
        T   = out(k);
        hit = 0;
        for i = 1:numel(T.t)
            if dhMergeable(T.v(i,:), vj, planarTwist)
                hit = i;
                break
            end
        end
        if hit == 0
            T.t(end+1)   = tj;
            T.v(end+1,:) = vj;
            newout(end+1) = dhCanon(T);                %#ok<AGROW>
        else
            [ta, va, ca] = dhProductToSum(T.t(hit), T.v(hit,:), tj, vj);
            for m = 1:2
                T2 = T;
                T2.t(hit)   = ta(m);
                T2.v(hit,:) = va(m,:);
                T2.c        = T2.c * ca(m);
                newout(end+1) = dhCanon(T2);           %#ok<AGROW>
            end
        end
    end
    out = newout;
end
out = dhCollect(out);
end


function C = dhMulTerms(A, B, planarTwist)
C = dhT0();
for p = 1:numel(A)
    for q = 1:numel(B)
        C = [C, dhMulOne(A(p), B(q), planarTwist)];    %#ok<AGROW>
    end
end
C = dhCollect(C);
end


function C = dhMatMul(A, B, planarTwist)
% 4x4 matrices whose entries are term lists.
C = cell(4,4);
for r = 1:4
    for c = 1:4
        acc = dhT0();
        for k = 1:4
            acc = dhAddTerms(acc, dhMulTerms(A{r,k}, B{k,c}, planarTwist));
        end
        C{r,c} = acc;
    end
end
end


function M = dhTermsMatText(T)
M = cell(4,4);
for r = 1:4
    for c = 1:4
        M{r,c} = dhTermsText(T{r,c});
    end
end
end


function txt = dhTermsText(T)
% One entry as text, e.g. 'C1*C23' or 'C1*a1 + C12*a2 + C123*a3'.
if isempty(T), txt = '0'; return; end

% order: fewest factors first, then the atoms, then the symbols
keys = cell(1,numel(T));
for k = 1:numel(T)
    parts = cell(1,numel(T(k).t));
    for i = 1:numel(T(k).t)
        parts{i} = dhAtomSortKey(T(k).t(i), T(k).v(i,:));
    end
    keys{k} = sprintf('%02d|%s|%s', numel(T(k).t), strjoin(parts,'+'), ...
        strjoin(T(k).s,'*'));
end
[~, ord] = sort(keys);
T = T(ord);

% factor an atom that every term has: C1*(a1 + C2*a2 + C23*a3)
facT = 0;
facV = [];
if numel(T) > 1
    for i = 1:numel(T(1).t)
        shared = true;
        for k = 2:numel(T)
            found = false;
            for m = 1:numel(T(k).t)
                if T(k).t(m) == T(1).t(i) && isequal(T(k).v(m,:), T(1).v(i,:))
                    found = true;
                    break
                end
            end
            if ~found
                shared = false;
                break
            end
        end
        if shared
            facT = T(1).t(i);
            facV = T(1).v(i,:);
            break
        end
    end
end
if facT ~= 0
    inner = dhTermsInner(dhDropAtom(T, facT, facV));
    if contains(inner, ' + ')
        txt = [dhAtomText(facT, facV) '*(' inner ')'];
    else
        txt = [dhAtomText(facT, facV) '*' inner];
    end
else
    txt = dhTermsInner(T);
end
end


function T = dhDropAtom(T, t, v)
for k = 1:numel(T)
    for i = 1:numel(T(k).t)
        if T(k).t(i) == t && isequal(T(k).v(i,:), v)
            T(k).t(i)   = [];
            T(k).v(i,:) = [];
            break
        end
    end
end
end


function txt = dhTermsInner(T)
parts = cell(1,numel(T));
for k = 1:numel(T)
    s = '';
    for i = 1:numel(T(k).t)
        a = dhAtomText(T(k).t(i), T(k).v(i,:));
        if isempty(s), s = a; else, s = [s '*' a]; end     %#ok<AGROW>
    end
    for m = 1:numel(T(k).s)
        if isempty(s)
            s = T(k).s{m};
        else
            s = [s '*' T(k).s{m}];                         %#ok<AGROW>
        end
    end
    if isempty(s), s = '1'; end
    ac = abs(T(k).c);
    if abs(ac - 1) > 1e-12
        s = sprintf('%.4g*%s', ac, s);
    end
    if T(k).c < 0
        s = ['-' s];
    end
    parts{k} = s;
end
txt = strjoin(parts, ' + ');
txt = strrep(txt, ' + -', ' - ');
end


function s = dhAtomText(t, v)
% 'C12' for cos(theta1 + theta2); a sum with mixed signs (rare) is written out
% as cos(theta2 - theta3), and then it needs no legend entry.
if t == 0
    s = '1';
    return
end
idx = find(v);
if isempty(idx)
    s = '1';
elseif all(v(idx) == 1)
    if t == 1, s = ['C' sprintf('%d',idx)]; else, s = ['S' sprintf('%d',idx)]; end
else
    ang = dhAngleText(v);
    if t == 1, s = ['cos(' ang ')']; else, s = ['sin(' ang ')']; end
end
end


function s = dhAngleText(v)
parts = {};
for k = 1:numel(v)
    if v(k) == 0, continue; end
    if abs(v(k)) == 1
        term = sprintf('theta%d', k);
    else
        term = sprintf('%d*theta%d', abs(v(k)), k);
    end
    if isempty(parts)
        if v(k) < 0, parts{1} = ['-' term]; else, parts{1} = term; end
    else
        if v(k) < 0
            parts{end+1} = ['- ' term];            %#ok<AGROW>
        else
            parts{end+1} = ['+ ' term];            %#ok<AGROW>
        end
    end
end
s = strjoin(parts, ' ');
end


function names = dhAtomsIn(T)
% The C.../S... abbreviations used by a term list, e.g. {'C1','S12'}.
% T may also be a matrix of term lists (a 4x4 cell), whose entries are
% searched one by one.
if iscell(T)
    names = {};
    for i = 1:numel(T)
        names = [names, dhAtomsIn(T{i})];          %#ok<AGROW>
    end
    names = unique(names);
    return
end
names = {};
for k = 1:numel(T)
    for i = 1:numel(T(k).t)
        idx = find(T(k).v(i,:));
        if isempty(idx) || any(T(k).v(i,idx) ~= 1), continue; end
        if T(k).t(i) == 1
            nm = ['C' sprintf('%d',idx)];
        else
            nm = ['S' sprintf('%d',idx)];
        end
        if ~any(strcmp(names, nm))
            names{end+1} = nm;                     %#ok<AGROW>
        end
    end
end
end


function M = dhEvalTerms(T, q, aVals, dVals, isRev)
% Numeric value of a term matrix at joint values q (rad / m), link lengths
% aVals (m) and the fixed offsets dVals (m). A prismatic joint's d_i is its
% moving variable, so it takes q(i) instead of the fixed offset.
M = zeros(4,4);
for r = 1:4
    for c = 1:4
        total = 0;
        terms = T{r,c};
        for k = 1:numel(terms)
            tk  = terms(k);
            val = tk.c;
            for i = 1:numel(tk.t)
                ang = tk.v(i,:) * q(:);
                if tk.t(i) == 1
                    val = val * cos(ang);
                else
                    val = val * sin(ang);
                end
            end
            for m = 1:numel(tk.s)
                nm  = tk.s{m};
                idx = str2double(nm(2:end));
                if nm(1) == 'a'
                    val = val * aVals(idx);
                elseif isRev(idx)
                    val = val * dVals(idx);
                else
                    val = val * q(idx);
                end
            end
            total = total + val;
        end
        M(r,c) = total;
    end
end
end

function txt = dhAbbrevText(names)
% 'C1 = cos(theta1)   S12 = sin(theta1 + theta2)   ...' for a list of names.
parts = cell(1,numel(names));
for k = 1:numel(names)
    nm = names{k};
    if nm(1) == 'S', fn = 'sin'; else, fn = 'cos'; end
    ds = nm(2:end);
    vs = cell(1,numel(ds));
    for q = 1:numel(ds)
        vs{q} = ['theta' ds(q)];
    end
    parts{k} = sprintf('%s = %s(%s)', nm, fn, strjoin(vs,' + '));
end
txt = strjoin(parts, '   ');
end


function out = dhAbbrevLines(names, width, prefix)
% Pack the legend entries into lines of at most `width` characters, never
% splitting an entry such as 'C12 = cos(theta1 + theta2)' across two lines.
if isempty(names)
    out = {};
    return;
end
parts  = strsplit(strtrim(dhAbbrevText(names)), '   ');
indent = repmat(' ', 1, max(numel(prefix), 4));
out = {prefix};
for k = 1:numel(parts)
    if numel(out{end}) + numel(parts{k}) + 3 > width
        out{end+1} = [indent parts{k} '   ']; %#ok<AGROW>
    else
        out{end} = [out{end} parts{k} '   '];
    end
end
for k = 1:numel(out)
    out{k} = regexprep(out{k}, '\s+$', '');
end
end


function B = dhSymBlock(SB, DH)
% The symbolic matrices as display rows, their titles, the shorthand legend
% and the common column width - everything the live readout needs.
% Section 7 calls this once; dhRebuild calls it again when a link length
% crosses zero, because a zero a_i changes the general form itself.
N = DH.N;
B.titles = cell(1,N+1);
B.rows   = cell(1,N+1);
for i = 1:N
    B.titles{i} = sprintf('T(%d -> %d)   [joint %d, %s]', ...
        i-1, i, i, DH.variable{i});
    B.rows{i} = dhSymRows(SB.TgenAbbr{i});
end
B.titles{N+1} = sprintf('T(0 -> %d)   (composed, simplified)', N);
B.rows{N+1}   = dhSymRows(SB.TtotAbbr);
B.abbrevLines = dhAbbrevLines(SB.abbrev, 158, 'shorthand:  ');

% Column width for the "symbolic | numeric" layout: one width for every block
% (per-joint AND composed), so all the '|' separators - the header included -
% line up. A row that still does not fit is wrapped, with the numeric value
% aligned underneath.
symW = 0;
for k = 1:numel(B.rows)
    for r = 1:numel(B.rows{k})
        symW = max(symW, numel(B.rows{k}{r}));
    end
end
B.symW = min(symW, 110);
end


function rows = dhSymRows(M)
% Render a matrix of general-form entries as text rows, with the entries of
% each column right-aligned so the commas line up. This is what is printed and
% what is shown in the control window.
%   M  a 4x4 cell of strings (the general form built by dhSymbolic) or a
%      numeric matrix (rendered by dhNum1).
if iscell(M)
    E = M;
else
    [r, k] = size(M);
    E = cell(r,k);
    for i = 1:r
        for j = 1:k
            E{i,j} = dhNum1(M(i,j));
        end
    end
end
[r, k] = size(E);
for i = 1:r
    for j = 1:k
        if ~ischar(E{i,j}), E{i,j} = char(E{i,j}); end
    end
end
w = zeros(1,k);
for j = 1:k
    w(j) = max(cellfun(@numel, E(:,j)));
end
rows = cell(1,r);
for i = 1:r
    parts = cell(1,k);
    for j = 1:k
        parts{j} = sprintf('%*s', w(j), E{i,j});
    end
    rows{i} = ['[ ' strjoin(parts, ', ') ' ]'];
end
end


function out = wrapLines(lines, width)
% Soft-wrap long text lines at spaces so they fit inside the readout list box
% (a list box clips, it does not wrap, so we wrap the text ourselves).
out = {};
for i = 1:numel(lines)
    s = lines{i};
    while numel(s) > width
        cut = find(s(1:width) == ' ', 1, 'last');
        if isempty(cut), cut = width; end
        out{end+1} = s(1:cut-1);          %#ok<AGROW>
        s = ['      ' s(cut+1:end)];
    end
    out{end+1} = s;                        %#ok<AGROW>
end
end


function s = dhNumRow(v)
% One numeric matrix row, 2 decimal places, aligned.
parts = cell(1,numel(v));
for i = 1:numel(v)
    parts{i} = sprintf('%7.2f', v(i));
end
s = ['[ ' strjoin(parts, ', ') ' ]'];
end


% ------------------------------------------------------- the physical robot
function robot = dhBuildRobot(DH, linkLengths)
% Build a rigidBodyTree from an already generated DH set, with physical
% geometry on every link.
N           = DH.N;
linkLengths = double(linkLengths(:))';

robot = rigidBodyTree('DataFormat','column','MaxNumBodies',N+1);

posLen = linkLengths(linkLengths > 0);
if isempty(posLen)
    rLink = 0.015;
else
    rLink = max(0.005, 0.070 * min(posLen));   % slim, so joint frames stay visible
end
rJoint = 1.15 * rLink;                          % joint sphere radius

parentName = robot.BaseName;
for i = 1:N
    if DH.isRev(i)
        jtype  = 'revolute';
        limits = [-pi pi];
        a_i    = linkLengths(i);
    else
        jtype  = 'prismatic';
        limits = [-linkLengths(i) linkLengths(i)];   % travel both ways
        a_i    = 0;
    end
    jnt = rigidBodyJoint(sprintf('joint%d',i), jtype);
    jnt.PositionLimits = limits;
    setFixedTransform(jnt, [a_i DH.alpha(i) DH.d(i) DH.theta(i)], 'dh');

    bod = rigidBody(sprintf('link%d',i));
    bod.Joint = jnt;

    % Physical geometry. The fixed DH transform of this joint at q = 0 gives
    % the parent origin seen from this body's frame: -R'*t (constant in q).
    Tfix  = dhTransform(a_i, DH.alpha(i), DH.d(i), DH.theta(i));
    pPar  = -Tfix(1:3,1:3)' * Tfix(1:3,4);
    Llink = norm(pPar - [0;0;0]);

    if Llink > 1e-6
        addVisual(bod, 'Cylinder', [rLink Llink], spanTransform(pPar,[0;0;0]));
    end
    addVisual(bod, 'Sphere', rJoint);

    % If the NEXT joint is prismatic, this body carries its slide rail.
    if i < N && ~DH.isRev(i+1)
        stroke = linkLengths(i+1);
        if stroke > 0
            addVisual(bod, 'Cylinder', [rLink/2 2*stroke], transTransform([0;0;0]));
        end
    end

    addBody(robot, bod, parentName);
    parentName = sprintf('link%d',i);
end
end


function T = dhTransform(a, alpha, d, theta)
% Standard DH homogeneous transformation matrix.
ct = cos(theta); st = sin(theta);
ca = cos(alpha); sa = sin(alpha);
T = [ct, -st*ca,  st*sa, a*ct;
     st,  ct*ca, -ct*sa, a*st;
      0,     sa,     ca,    d;
      0,      0,      0,    1];
end


function H = transTransform(v)
% Pure translation as a 4x4 homogeneous transform.
H = eye(4);
H(1:3,4) = v(:);
end


function H = spanTransform(p0, p1)
% Place a unit cylinder (axis along local z, centred) so it spans p0..p1.
p0 = p0(:); p1 = p1(:);
d  = p1 - p0;
L  = norm(d);
mid = (p0 + p1)/2;
if L < 1e-12
    R = eye(3);
else
    R = rotFromZ(d/L);
end
H = [R, mid; 0 0 0 1];
end


function R = rotFromZ(zhat)
% Rotation matrix that maps the z axis [0;0;1] onto the unit vector zhat.
zhat = zhat(:);
z0   = [0;0;1];
v    = cross(z0, zhat);
c    = dot(z0, zhat);
s    = norm(v);
if s < 1e-12
    if c >= 0
        R = eye(3);
    else
        R = [1 0 0; 0 -1 0; 0 0 -1];   % 180 deg about x
    end
else
    vx = [0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0];
    R  = eye(3) + vx + vx*vx*((1-c)/s^2);   % Rodrigues
end
end


% ------------------------------------------------------------- the graphics
function applyView(ax, reach, zl)
% Zoom the axes to the robot's reach so it fills the view, with equal scales.
% axis('equal') must come FIRST: it re-fits the limits, so it would undo any
% limits set before it. Setting the limits afterwards keeps the view stable
% while the robot moves.
%
% zl (optional) is the z interval to display, from dhPlotZRange. For a planar
% arm it is a thin slab; a cube would waste most of the axes on empty space
% above and below a flat arm.
if ~(reach > 0), reach = 1; end
lim = 1.18 * reach;      % slack so the projections and their labels fit inside
if nargin < 3 || isempty(zl), zl = [-lim lim]; end
axis(ax,'equal');
xlim(ax, [-lim lim]); ylim(ax, [-lim lim]); zlim(ax, zl);
grid(ax,'on');
end


function dhSetPlaneView(fig, which)
% Camera straight down one coordinate plane, in ORTHOGRAPHIC projection so the
% drawing is a true projection of that plane (a perspective camera would
% foreshorten the far parts of the arm):
%
%   XY  from +z : x to the right, y up
%   YZ  from +x : y to the right, z up
%   ZX  from -y : x to the right, z up
%   3D  back to the usual perspective view - the way out of a plane view
S = getappdata(fig,'dhstate');
if isempty(S) || ~isvalid(fig), return; end
which = upper(which);

switch which
    case 'XY'
        view(S.ax, 0, 90);
        camproj(S.ax, 'orthographic');
        S.planeView = 'XY';
    case 'YZ'
        view(S.ax, 90, 0);
        camproj(S.ax, 'orthographic');
        S.planeView = 'YZ';
    case 'ZX'
        view(S.ax, 0, 0);
        camproj(S.ax, 'orthographic');
        S.planeView = 'ZX';
    otherwise                    % '3D' - the perspective view we started in
        view(S.ax, 3);
        camproj(S.ax, 'perspective');
        S.planeView = '';
end

setappdata(fig,'dhstate',S);
dhMarkViewButton(fig, S.planeView);
end


function dhMarkViewButton(fig, active)
% Highlight the button of the view that is on screen ('3D' while in the 3-D
% view), so it is always obvious where you are and how to get back.
bt = findall(fig, 'Tag','viewbutton');
for k = 1:numel(bt)
    isOn = strcmpi(get(bt(k),'String'), active) || ...
        (isempty(active) && strcmpi(get(bt(k),'String'),'3D'));
    if isOn
        set(bt(k), 'BackgroundColor',[0.82 0.90 1.00]);
    else
        set(bt(k), 'BackgroundColor',[0.94 0.94 0.94]);
    end
end
end


function zl = dhPlotZRange(robot, DH, L, isPlanar, reach)
% The z interval the plot should show.
%
% A SPATIAL arm really does sweep a cube, so it gets one.
%
% A PLANAR arm (every DH twist a multiple of 180 deg) keeps all of its joints
% in ONE horizontal plane, and that height does not depend on the revolute
% angles at all - it is the sum of the fixed offsets and the prismatic strokes.
% Drawing a cube for such an arm wastes most of the axes height on the empty
% space above and below the plane; a thin slab of the height the arm actually
% uses lets MATLAB draw the arm about twice as large, and the workspace then
% fills the width of the plot band as well.
%
% The height is found by evaluating the chain at the ends of every prismatic
% stroke (at most 2^4 = 16 configurations), so the slab always contains the
% whole arm - a raised base, a vertical stroke and a planar arm with offsets
% all included. The base frame sits at z = 0.
N = DH.N;
if ~(reach > 0), reach = 1; end
if ~isPlanar
    zl = [-1.18 1.18] * reach;
    return;
end

zmin = 0;  zmax = 0;
idxP = find(~DH.isRev);
nP   = numel(idxP);
for m = 0:(2^nP - 1)
    q = zeros(N,1);
    for b = 1:nP
        % a slide travels both ways: its extremes are -stroke and +stroke
        q(idxP(b)) = L(idxP(b)) * (2*bitget(m, b) - 1);
    end
    for k = 1:N
        zk   = getTransform(robot, q, sprintf('link%d',k));
        zmin = min(zmin, zk(3,4));
        zmax = max(zmax, zk(3,4));
    end
end

pad = max(0.25*reach, 0.05);      % margin so nothing touches the slab faces
zl  = [zmin - pad, zmax + pad];
end


function [posX, posY, posZ] = dhProjectionLabelPos(pEnd, reach)
% Where the projected x, y and z values are written. Shared by the live window
% and the static home figure so the two pictures agree.
%
% x and y sit just outside their feet on the X and Y axes. z sits a little
% ABOVE the end point: the z axis is the vertical one on screen, so a pure +z
% offset always lifts that label clear of the marker and of the arm, whether
% the arm is planar (z = 0, the vertical drop has zero length) or spatial.
if ~(reach > 0), reach = 1; end
x = pEnd(1); y = pEnd(2); z = pEnd(3);
m = 0.05 * reach;

posX = [x, -m, 0];
posY = [-m, y, 0];
posZ = [x,  y, z + 0.055*reach];
end


function dhDrawProjections(ax, pEnd, reach)
% Dashed projections of the end point onto the coordinate axes, with the
% projected x, y and z values written beside them - the same picture the live
% window shows, drawn once here for the static home figure.
%
%   P -> (x,y,0)      the vertical drop (z)
%   (x,y,0) -> (x,0,0) ends on the X axis (x)
%   (x,y,0) -> (0,y,0) ends on the Y axis (y)
if reach <= 0, reach = 1; end
x = pEnd(1); y = pEnd(2); z = pEnd(3);
Pf = [x y 0];
Px = [x 0 0];
Py = [0 y 0];
cols = {[0.85 0.10 0.10], [0.10 0.65 0.10], [0.10 0.10 0.85]};

plot3(ax, [x Pf(1) nan Pf(1) Px(1) nan Pf(1) Py(1)], ...
          [y Pf(2) nan Pf(2) Px(2) nan Pf(2) Py(2)], ...
          [z Pf(3) nan Pf(3) Px(3) nan Pf(3) Py(3)], ':', ...
          'Color',[0.45 0.45 0.45], 'LineWidth',1.0);
plot3(ax, Px(1),Px(2),Px(3), 'o', 'MarkerSize',5, ...
      'MarkerFaceColor',cols{1}, 'MarkerEdgeColor','none');
plot3(ax, Py(1),Py(2),Py(3), 'o', 'MarkerSize',5, ...
      'MarkerFaceColor',cols{2}, 'MarkerEdgeColor','none');
plot3(ax, Pf(1),Pf(2),Pf(3), 'o', 'MarkerSize',5, ...
      'MarkerFaceColor',cols{3}, 'MarkerEdgeColor','none');

[posX, posY, posZ] = dhProjectionLabelPos(pEnd, reach);
text(ax, posX(1),posX(2),posX(3), sprintf('x = %.2f m',x), ...
    'Color',[0.25 0.25 0.25], 'FontSize',8, ...
    'HorizontalAlignment','center', 'VerticalAlignment','top');
text(ax, posY(1),posY(2),posY(3), sprintf('y = %.2f m',y), ...
    'Color',[0.25 0.25 0.25], 'FontSize',8, ...
    'HorizontalAlignment','right', 'VerticalAlignment','middle');
text(ax, posZ(1),posZ(2),posZ(3), sprintf('z = %.2f m',z), ...
    'Color',[0.25 0.25 0.25], 'FontSize',8, ...
    'HorizontalAlignment','center', 'VerticalAlignment','bottom');
end


function dhDrawFrames(ax, Tframes, reach)
% Redraw the coordinate frame at every joint origin (x red, y green, z blue)
% as THICK lines on top of the robot, so the frames stay visible even when the
% physical links are thick enough to hide the built-in thin frames.
if reach <= 0, reach = 1; end
len = 0.09 * reach;
cols = {[0.85 0.10 0.10], [0.10 0.65 0.10], [0.10 0.10 0.85]};
for k = 1:numel(Tframes)
    p  = Tframes{k}(1:3,4);
    Rk = Tframes{k}(1:3,1:3);
    for a = 1:3
        v = Rk(:,a) * len;
        plot3(ax, [p(1) p(1)+v(1)], [p(2) p(2)+v(2)], [p(3) p(3)+v(3)], ...
            '-', 'Color',cols{a}, 'LineWidth',1.8);
    end
end
end


function dhTidyPlot(fig)
% Keep the whole plot inside its own band of the window.
%
% Two things need housekeeping:
%  1. show() adds a small 3-axis orientation triad as its own tiny axes
%     (tagged CornerCoordinateFrame). Left alone it sits in the figure's
%     bottom-left corner - on top of the slider column and the readout. We
%     move it inside the plot area instead.
%  2. Text objects default to Clipping 'off', so they can be drawn anywhere on
%     the figure (including over the controls) when you zoom in. Clipping
%     everything to the axes makes the plot band a hard boundary.
triad = findall(fig, 'Type','axes', 'Tag','CornerCoordinateFrame');
for k = 1:numel(triad)
    try
        set(triad(k), 'Units','normalized', ...
            'Position',[0.295 0.580 0.085 0.085], 'Color','none');
    catch
    end
end

% clip every object of the main (Primary) axes
axAll = findall(fig, 'Type','axes');
for k = 1:numel(axAll)
    if strcmp(get(axAll(k),'Tag'),'CornerCoordinateFrame'), continue; end
    try
        cl = findall(axAll(k), '-property','Clipping');
        % the view buttons must stay unclipped: they sit at the very edge
        cl = cl(~strcmp(get(cl,'Tag'),'viewbutton'));
        set(cl, 'Clipping','on');
    catch
        set(findall(axAll(k),'Type','text'), 'Clipping','on');
        set(findall(axAll(k),'Type','line'), 'Clipping','on');
    end
end
end


function G = dhCreateScene(ax, N)
% Create the whole scene ONCE and hand back its handles.
%
% Creating graphics objects is what costs time in MATLAB: roughly 20 ms per
% plot3 call and per patch in a figure this size. Re-creating the robot's mesh
% every frame (what show() has to do) therefore costs ~170 ms per frame, about
% 6 frames per second. By creating the objects exactly once, the animation
% below only has to write coordinates into existing objects.
%
% The whole robot is only THREE patches (all links, all rails, all joints) and
% three lines (one per frame axis): writing one big array is much cheaper than
% notifying twenty small objects, so there are very few property updates per
% frame.
%
% Meshes are UNIT shapes (a cylinder of radius 1 and height 1, a sphere of
% radius 1); the actual size is applied by dhSetGeometry when the link lengths
% change, so nothing has to be re-created for the geometry either.
[Vc, Fc] = dhCylinderMesh(16);
[Vs, Fs] = dhSphereMesh(10);
G.nv = size(Vc,1);                       % vertices per cylinder
G.ns = size(Vs,1);                       % vertices per sphere
G.linkV  = cell(1,N);                    % body-frame vertices (dhSetGeometry)
G.railV  = cell(1,N);
G.colV   = cell(1,N);

G.grey     = [0.62 0.62 0.62];
G.frameCol = {[0.85 0.10 0.10], [0.10 0.65 0.10], [0.10 0.10 0.85]};

% one patch holding all N links (each copy of the cylinder is index-shifted)
G.link = patch('Parent',ax,'Faces',dhTileFaces(Fc,N), ...
    'Vertices',zeros(N*G.nv,3),'FaceColor',G.grey,'EdgeColor','none', ...
    'Clipping','on');

% one patch holding the N pedestals that stand on the base (the fixed d_i).
% This is what turns a raised base into a COLUMN under the arm instead of a
% diagonal strut from the floor to the first joint.
G.col = patch('Parent',ax,'Faces',dhTileFaces(Fc,N), ...
    'Vertices',zeros(N*G.nv,3),'FaceColor',G.grey*0.82,'EdgeColor','none', ...
    'Clipping','on');

% one patch holding all N rails; unused slots collapse to a point
G.rail = patch('Parent',ax,'Faces',dhTileFaces(Fc,N), ...
    'Vertices',zeros(N*G.nv,3),'FaceColor',G.grey*0.92,'EdgeColor','none', ...
    'Clipping','on','Visible','off');

% one patch holding the base joint plus one joint per body
G.joint = patch('Parent',ax,'Faces',dhTileFaces(Fs,N+1), ...
    'Vertices',zeros((N+1)*G.ns,3),'FaceColor',[0.50 0.50 0.50], ...
    'EdgeColor','none','Clipping','on');

% thick XYZ frames at every joint origin: one NaN-separated polyline per axis
% (thin built-in frames would be hidden behind the physical links)
G.frame = gobjects(1,3);
for a = 1:3
    G.frame(a) = plot3(ax, nan(1,3*(N+1)), nan(1,3*(N+1)), nan(1,3*(N+1)), ...
        '-','Color',G.frameCol{a},'LineWidth',1.8,'Clipping','on');
end

% end-effector marker and the projections of its coordinates onto the axes
G.marker = plot3(ax,0,0,0,'o','MarkerSize',11,'LineWidth',1.6, ...
    'MarkerFaceColor',[0.85 0.10 0.10],'MarkerEdgeColor','w', ...
    'Clipping','on');

% the projection lines as ONE NaN-separated object (a single property update
% per frame). Three segments:
%   P  -> (x,y,0)     the vertical drop, i.e. the z extent
%   (x,y,0) -> (x,0,0) the y extent, ending on the X axis
%   (x,y,0) -> (0,y,0) the x extent, ending on the Y axis
G.proj = plot3(ax, nan(1,9), nan(1,9), nan(1,9), ':', ...
    'Color',[0.45 0.45 0.45],'LineWidth',1.0,'Clipping','on');

% the feet of the projections, coloured like the axis they belong to
G.foot = gobjects(1,3);
for a = 1:3
    G.foot(a) = plot3(ax,0,0,0,'o','LineStyle','none','MarkerSize',5, ...
        'MarkerFaceColor',G.frameCol{a},'MarkerEdgeColor','none', ...
        'Clipping','on');
end

% the projected values, written next to the projection lines (they are only
% re-written with the readout, ~10 Hz, not on every animation frame)
G.ptxt = gobjects(1,3);
for a = 1:3
    G.ptxt(a) = text(ax,0,0,0,' ','Color',[0.25 0.25 0.25],'FontSize',8, ...
        'Interpreter','none','Clipping','on', ...
        'HorizontalAlignment','center','VerticalAlignment','middle');
end
end


function Ft = dhTileFaces(F, count)
% Repeat a face list `count` times, shifting the vertex indices of each copy so
% that one patch can hold `count` identical meshes.
nv   = max(F(:));
m    = size(F,1);
Ft   = repmat(F, count, 1) + repelem((0:count-1).' * nv, m, 1);
end


function G = dhSetGeometry(G, DH, L)
% Work out the body-frame geometry from the current link lengths. Nothing is
% created or deleted here - only vertex templates are recomputed - so dragging
% a LINK LENGTH slider is smooth as well.
%
% For body i the link runs from the parent origin to this body's origin; seen
% from this body's frame that parent origin sits at -R'*t of the fixed DH
% transform (constant in q), which is exactly what dhBuildRobot uses.
% G.linkV{i} (or G.railV{i}) is empty when this body has no link (or no rail);
% dhUpdateVisuals then collapses that slot onto a single point.
N = DH.N;
posLen = L(L > 0);
if isempty(posLen)
    rLink = 0.015;
else
    rLink = max(0.005, 0.070 * min(posLen));    % slim, so frames stay visible
end

[Vc, ~] = dhCylinderMesh(16);
for i = 1:N
    if DH.isRev(i)
        a_i = L(i);
    else
        a_i = 0;
    end
    Tfix  = dhTransform(a_i, DH.alpha(i), DH.d(i), DH.theta(i));
    pPar  = -Tfix(1:3,1:3).' * Tfix(1:3,4);

    % The fixed transform holds TWO different objects, and they must not be
    % drawn as one:
    %   d_i -> the pedestal that rises along the PARENT z axis. For joint 1
    %          that is the base column (a SCARA's shoulder height); for a
    %          later joint it is the offset between parallel axes.
    %   a_i -> the link that runs out of the TOP of that pedestal.
    % Drawing a single cylinder from the parent origin to this body's origin
    % turned the pedestal into a diagonal strut, so a raised base looked as if
    % the first link were bolted to the floor.
    pTop       = Tfix(1:3,1:3).' * ([0;0;DH.d(i)] - Tfix(1:3,4));
    G.colV{i}  = dhCylBetween(Vc, 1.8*rLink, pPar,  pTop);   % pedestal (d_i)
    G.linkV{i} = dhCylBetween(Vc, rLink,     pTop, [0;0;0]); % link     (a_i)

    % the rail of a following prismatic joint lies in THIS body's frame. The
    % slide travels both ways, so the rail is centred on the zero position and
    % is 2*stroke long.
    if i < N && ~DH.isRev(i+1) && L(i+1) > 0
        stroke = L(i+1);
        G.railV{i} = (Vc .* [rLink/2 rLink/2 2*stroke]);
    else
        G.railV{i} = [];
    end
end

G.jointV = dhSphereMesh(10) * (1.15 * rLink);   % vertices of one scaled sphere
G.rLink  = rLink;
end


function V = dhCylBetween(Vc, r, p0, p1)
% A cylinder of radius r from p0 to p1, as body-frame vertices ([] if the two
% points coincide, so the slot collapses and nothing is drawn).
v  = p1(:) - p0(:);
Ln = norm(v);
if Ln < 1e-9
    V = [];
    return;
end
R = rotFromZ(v / Ln);
V = (Vc .* [r r Ln]) * R.' + ((p0(:) + p1(:))/2).';
end


function dhUpdateVisuals(S, Tframes, pEnd, reach)
% Move the whole scene to the configuration in Tframes. This is the function
% that runs on every animation frame, so it only writes coordinates into the
% few objects created by dhCreateScene: one patch of links, one patch of rails,
% one patch of joint spheres, three frame lines, the end marker and the three
% coordinate-projection lines.
G = S.g;
N = S.N;
if ~(reach > 0), reach = 1; end
len = 0.09 * reach;                      % arm length of the XYZ frames
nv  = G.nv;
ns  = G.ns;

% --- all links in one patch ---------------------------------------------
Vl = zeros(N*nv,3);
for i = 1:N
    pi = Tframes{i+1}(1:3,4);
    r  = (i-1)*nv + (1:nv);
    if isempty(G.linkV{i})
        Vl(r,:) = repmat(pi.', nv, 1);   % collapsed: nothing is drawn
    else
        Ri = Tframes{i+1}(1:3,1:3);
        Vl(r,:) = G.linkV{i}*Ri.' + pi.';
    end
end
set(G.link,'Vertices',Vl);

% --- all base columns (pedestals) in one patch --------------------------
Vk = zeros(N*nv,3);
for i = 1:N
    pi = Tframes{i+1}(1:3,4);
    r  = (i-1)*nv + (1:nv);
    if isempty(G.colV{i})
        Vk(r,:) = repmat(pi.', nv, 1);
    else
        Ri = Tframes{i+1}(1:3,1:3);
        Vk(r,:) = G.colV{i}*Ri.' + pi.';
    end
end
set(G.col,'Vertices',Vk);

% --- all slide rails in one patch ---------------------------------------
Vr = zeros(N*nv,3);
anyRail = false;
for i = 1:N
    pi = Tframes{i+1}(1:3,4);
    r  = (i-1)*nv + (1:nv);
    if isempty(G.railV{i})
        Vr(r,:) = repmat(pi.', nv, 1);
    else
        Ri = Tframes{i+1}(1:3,1:3);
        Vr(r,:) = G.railV{i}*Ri.' + pi.';
        anyRail = true;
    end
end
if anyRail
    set(G.rail,'Vertices',Vr,'Visible','on');
else
    set(G.rail,'Visible','off');
end

% --- the base joint plus one joint per body, in one patch ---------------
Vj = zeros((N+1)*ns,3);
for k = 1:N+1
    p = Tframes{k}(1:3,4);
    Vj((k-1)*ns + (1:ns), :) = G.jointV + p.';
end
set(G.joint,'Vertices',Vj);

% --- thick XYZ frame at every joint origin (one NaN-separated line per
%     axis, so this is three property updates instead of 3*(N+1)) ---------
Xd = nan(3,3*(N+1)); Yd = Xd; Zd = Xd;
for k = 1:N+1
    p   = Tframes{k}(1:3,4);
    Rk  = Tframes{k}(1:3,1:3);
    idx = 3*(k-1);
    for a = 1:3
        v = Rk(:,a) * len;
        Xd(a, idx+1:idx+2) = [p(1) p(1)+v(1)];
        Yd(a, idx+1:idx+2) = [p(2) p(2)+v(2)];
        Zd(a, idx+1:idx+2) = [p(3) p(3)+v(3)];
    end
end
for a = 1:3
    set(G.frame(a), 'XData',Xd(a,:), 'YData',Yd(a,:), 'ZData',Zd(a,:));
end

% end marker and the projection of its coordinates onto the axes ---------
set(G.marker,'XData',pEnd(1),'YData',pEnd(2),'ZData',pEnd(3));

x = pEnd(1); y = pEnd(2); z = pEnd(3);
Pf = [x y 0];                            % foot on the XY plane
Px = [x 0 0];                            % foot on the X axis
Py = [0 y 0];                            % foot on the Y axis
set(G.proj, 'XData',[x Pf(1) nan Pf(1) Px(1) nan Pf(1) Py(1)], ...
            'YData',[y Pf(2) nan Pf(2) Px(2) nan Pf(2) Py(2)], ...
            'ZData',[z Pf(3) nan Pf(3) Px(3) nan Pf(3) Py(3)]);
set(G.foot(1),'XData',Px(1),'YData',Px(2),'ZData',Px(3));   % x on the X axis
set(G.foot(2),'XData',Py(1),'YData',Py(2),'ZData',Py(3));   % y on the Y axis
set(G.foot(3),'XData',Pf(1),'YData',Pf(2),'ZData',Pf(3));   % z on the plane
end


function dhUpdateProjectionText(S, pEnd, reach)
% Write the projected x, y and z values beside their projection lines. This
% runs with the readout (about 10 times a second), not on every animation
% frame: moving the lines is cheap, repainting text is not.
G = S.g;
if ~(reach > 0), reach = 1; end
x = pEnd(1); y = pEnd(2); z = pEnd(3);
[posX, posY, posZ] = dhProjectionLabelPos(pEnd, reach);

% x: under its foot on the X axis (pushed away from the arm)
set(G.ptxt(1), 'Position',posX, 'String',sprintf('x = %.2f m',x), ...
    'HorizontalAlignment','center', 'VerticalAlignment','top');
% y: left of its foot on the Y axis
set(G.ptxt(2), 'Position',posY, 'String',sprintf('y = %.2f m',y), ...
    'HorizontalAlignment','right', 'VerticalAlignment','middle');
% z: just above the end point, beside the vertical drop
set(G.ptxt(3), 'Position',posZ, 'String',sprintf('z = %.2f m',z), ...
    'HorizontalAlignment','center', 'VerticalAlignment','bottom');
end


function [V, F] = dhCylinderMesh(ns)
% Unit cylinder: radius 1, height 1, axis along +z, centred on the origin.
% Vertices 1..ns are the bottom ring, ns+1..2ns the top ring, then the two
% cap centres. Every face index stays within the vertex list.
th = linspace(0,2*pi,ns+1).'; th(end) = [];
x = cos(th); y = sin(th);
V = [x y -0.5*ones(ns,1); x y 0.5*ones(ns,1); 0 0 -0.5; 0 0 0.5];
n = 2*ns;
F = zeros(3*ns,4);
for i = 1:ns
    j = mod(i,ns) + 1;
    F(i,:)        = [i j ns+j ns+i];        % side quad
    F(ns+i,:)     = [n+1 j i i];            % bottom cap (degenerate quad)
    F(2*ns+i,:)   = [n+2 ns+i ns+j ns+j];   % top cap
end
end


function [V, F] = dhSphereMesh(n)
% Unit sphere (radius 1) as a quad mesh built on MATLAB's sphere() grid. The
% quads at the poles are degenerate, which is harmless for a patch.
[X,Y,Z] = sphere(n);
V = [X(:) Y(:) Z(:)];
F = zeros(n*n,4);
for i = 1:n
    for j = 1:n
        v = (i-1)*(n+1) + j;
        F((i-1)*n+j,:) = [v v+1 v+n+2 v+n+1];
    end
end
end


% ------------------------------------------------------------- the live UI
function [q, L] = dhReadQL(fig)
% Read the sliders and convert to SI: q in radians (revolute) or metres
% (prismatic); L in metres. The UI itself is in degrees and millimetres.
S = getappdata(fig,'dhstate');
N = S.N;
q = zeros(N,1);
L = zeros(1,N);
for i = 1:N
    if S.isRev(i)
        q(i) = S.jointSliders(i).Value * pi/180;   % deg -> rad
    else
        q(i) = S.jointSliders(i).Value / 1000;     % mm  -> m
    end
    L(i) = S.lengthSliders(i).Value / 1000;        % mm  -> m
end
end


function dhSyncEdits(fig)
% Refresh every text box so it shows the current slider value. Only boxes that
% actually changed are written: setting a uicontrol's String costs a repaint.
S = getappdata(fig,'dhstate');
if ~isvalid(fig), return; end
for i = 1:S.N
    s = sprintf('%.1f', S.jointSliders(i).Value);
    if ~strcmp(get(S.jointEdit(i),'String'), s)
        set(S.jointEdit(i),'String',s);
    end
    s = sprintf('%.0f', S.lengthSliders(i).Value);
    if ~strcmp(get(S.lengthEdit(i),'String'), s)
        set(S.lengthEdit(i),'String',s);
    end
end
end


function dhRedraw(fig, force)
% Forward kinematics for the current configuration, redraw the robot, and
% refresh the readout. Does NOT rebuild the robot.
%
% The pose is updated on EVERY call, but the two text-heavy parts - the plot
% title and the 30-line readout list box - are throttled to ~10 Hz unless
% force is true (a typed value, or a slider release, passes force = true).
% Repainting that list box every frame is a large part of what makes an
% otherwise fast update feel sluggish.
if nargin < 2, force = false; end
S = getappdata(fig,'dhstate');
if ~isvalid(fig), return; end
N = S.N;

[q, L] = dhReadQL(fig);

Tee        = getTransform(S.robot, q, sprintf('link%d',N));
pEnd       = Tee(1:3,4);
Tframes    = cell(1,N+1);                     % J0 is the base frame
Tframes{1} = eye(4);
origins    = zeros(3,N+1);
for i = 1:N
    Ti = getTransform(S.robot, q, sprintf('link%d',i));
    Tframes{i+1}   = Ti;
    origins(:,i+1) = Ti(1:3,4);
end

% move the pre-built scene to this pose. No graphics object is created here,
% so this is fast enough to run continuously while a slider is dragged.
dhUpdateVisuals(S, Tframes, pEnd, sum(L));

% the title and the readout only follow at ~10 Hz (see the comment above)
doText = force || toc(S.tLastReadout) > 0.10;
if ~doText
    S.textStale = true;               % a later idle tick will settle it
    setappdata(fig,'dhstate',S);
    drawnow limitrate;
    return;
end
S.tLastReadout = tic;
S.textStale   = false;
setappdata(fig,'dhstate',S);

title(S.ax, sprintf('%d-DOF %s %s   |   end = (%.2f, %.2f, %.2f) m', ...
    N, upper(S.jointSeq), S.typeStr, pEnd));

% the projected coordinate values follow the readout (same ~10 Hz)
dhUpdateProjectionText(S, pEnd, sum(L));

% ---- readout: joint values, end position, frame origins -----------------
qParts = cell(1,N);
for i = 1:N
    if S.isRev(i)
        qParts{i} = sprintf('%+.1f deg', q(i)*180/pi);
    else
        qParts{i} = sprintf('%+.1f mm',  q(i)*1000);
    end
end
lines = {sprintf('q = [ %s ]', strjoin(qParts,'  '))};
lines{end+1} = sprintf('End effector  (x, y, z) = (%.2f, %.2f, %.2f) m', pEnd);
jstr = '';
for i = 0:N
    jstr = [jstr sprintf('J%d (%.2f, %.2f, %.2f)   ', i, origins(:,i+1))]; %#ok<AGROW>
end
lines{end+1} = strtrim(jstr);
lines{end+1} = '';

% ---- readout: symbolic matrix NEXT TO its numeric value (2 d.p.) --------
lines{end+1} = sprintf('%-*s | %s', S.symW, ...
    'MATRIX (symbolic general form)', 'NUMERIC (2 d.p.)');
if ~isempty(S.abbrevLines)
    lines = [lines, S.abbrevLines];
end
for k = 1:numel(S.symTitles)
    if k <= N
        % The symbolic block is the LOCAL matrix T(k-1 -> k) - that is what
        % dhJointGeneral builds - so the numeric value must be the local one
        % too. Tframes{k+1} is the transform of link k in the BASE frame
        % (T(0 -> k)); showing it beside T(k-1 -> k) made every block after
        % the first compare a local matrix with a global one. For an RPP arm
        % that was glaring: T(1 -> 2) is a bare slide, while T(0 -> 2) still
        % carries joint 1's rotation.
        M = Tframes{k} \ Tframes{k+1};    % T(k-1 -> k)
    else
        M = Tee;                          % composed T(0 -> N)
    end
    lines{end+1} = S.symTitles{k};                                   %#ok<AGROW>
    rows = S.symRows{k};
    for r = 1:numel(rows)
        nrow = dhNumRow(M(r,:));
        cand = sprintf('%-*s | %s', S.symW, rows{r}, nrow);
        if numel(cand) <= 158
            lines{end+1} = cand;                                     %#ok<AGROW>
        else
            % long symbolic row: wrap it, then align the numeric value under
            % the same column as the shorter rows
            wr = wrapLines({rows{r}}, 158);
            for q = 1:numel(wr)
                lines{end+1} = wr{q};                                %#ok<AGROW>
            end
            lines{end+1} = sprintf('%*s | %s', S.symW, '', nrow);    %#ok<AGROW>
        end
    end
end

set(S.readout,'String',lines,'Value',[],'ListboxTop',1);
drawnow limitrate;
end


function dhRebuild(fig)
% Rebuild the robot from the current link lengths and redraw. Also keeps the
% prismatic slider ranges in step with the current stroke.
S = getappdata(fig,'dhstate');
if ~isvalid(fig), return; end

[~, L] = dhReadQL(fig);
DH   = dhGenerate(S.jointSeq, L, S.jointAlpha, S.baseOffsets);
aWas = (abs(S.DH.a) > 0);                    % which links were non-zero

for i = 1:S.N
    if ~S.isRev(i)
        hi = max(L(i)*1000, 0.001);          % keep Max > Min even at zero
        sl = S.jointSliders(i);
        if sl.Max ~= hi || sl.Min ~= -hi
            v = sl.Value;
            sl.Min = -hi;                    % a slide goes both ways
            sl.Max =  hi;
            sl.Value = min(max(v, sl.Min), sl.Max);
        end
    end
end

S.DH    = DH;
S.robot = dhBuildRobot(DH, L);
S.g     = dhSetGeometry(S.g, DH, L);       % vertex templates for the new lengths

% A link slid down to zero disappears from the GENERAL form (its column is 0,
% not 'a1*cos(...)'), and a link that grows back reappears. So the symbolic
% text is rebuilt whenever the set of non-zero link lengths changes - once per
% crossing, not on every frame of the drag.
if ~isequal(aWas, (abs(DH.a) > 0))
    symBlock      = dhSymBlock(dhSymbolic(DH), DH);
    S.symTitles   = symBlock.titles;
    S.symRows     = symBlock.rows;
    S.abbrevLines = symBlock.abbrevLines;
    S.symW        = symBlock.symW;
end

setappdata(fig,'dhstate',S);

reach = sum(L);
applyView(S.ax, reach, ...
    dhPlotZRange(S.robot, DH, L, S.isPlanar, reach));   % re-fit once, reach changed
dhRedraw(fig, true);
end


function dhPoll(fig, force)
% Continuous-update poll: while a slider is dragged its Value changes, so this
% detects the change and animates the robot. Called by the timer (force false)
% and by the release callbacks of the sliders (force true, which also refreshes
% the readout straight away).
if nargin < 2, force = false; end
if ~isvalid(fig), return; end
S = getappdata(fig,'dhstate');
if isempty(S) || S.animating, return; end

curJ = zeros(1,S.N); curL = zeros(1,S.N);
for i = 1:S.N
    curJ(i) = S.jointSliders(i).Value;
    curL(i) = S.lengthSliders(i).Value;
end
chJ = any(abs(curJ - S.lastJoint) > 1e-6);
chL = any(abs(curL - S.lastLen)   > 1e-6);

if chL
    % lengths changed: rebuild, but not more often than every 50 ms
    if toc(S.tLastRebuild) < 0.05 && ~force, return; end
    S.tLastRebuild = tic;
    S.lastLen = curL;  S.lastJoint = curJ;
    setappdata(fig,'dhstate',S);
    dhRebuild(fig);
    dhSyncEdits(fig);
elseif chJ
    % only joint values changed: redraw the pose (fast, no rebuild)
    S.lastJoint = curJ;
    setappdata(fig,'dhstate',S);
    dhSyncEdits(fig);
    dhRedraw(fig, force);
elseif force
    dhRedraw(fig, true);      % settle the readout when the slider is released
elseif S.textStale
    % motion has stopped and the last pose frame skipped the text: do it once
    S.textStale = false;
    setappdata(fig,'dhstate',S);
    dhRedraw(fig, true);
end
end


function dhLengthRelease(fig)
% A link-length slider was released: finalise the rebuild.
if ~isvalid(fig), return; end
S = getappdata(fig,'dhstate');
for i = 1:S.N
    S.lastLen(i)   = S.lengthSliders(i).Value;
    S.lastJoint(i) = S.jointSliders(i).Value;
end
S.tLastRebuild = tic;
setappdata(fig,'dhstate',S);
dhRebuild(fig);
dhSyncEdits(fig);
end


function dhJointEdit(fig, i)
% A value was typed into a joint text box: clamp it, then animate the joint
% smoothly from its current value to the new one.
S = getappdata(fig,'dhstate');
if ~isvalid(fig), return; end
v  = str2double(get(S.jointEdit(i),'String'));
sl = S.jointSliders(i);
if isnan(v)
    dhSyncEdits(fig);          % not a number: put the old value back
    return;
end
v = min(max(v, sl.Min), sl.Max);
from = sl.Value;

S.animating = true;  setappdata(fig,'dhstate',S);
nFrames = 15;
for k = 1:nFrames
    if ~isvalid(fig), return; end
    sl.Value = from + (v - from) * (k/nFrames);
    dhRedraw(fig);
end

S = getappdata(fig,'dhstate');
S.animating = false;
S.lastJoint(i) = v;
setappdata(fig,'dhstate',S);
dhRedraw(fig, true);           % settle the readout on the final value
dhSyncEdits(fig);
end


function dhLengthEdit(fig, i)
% A value was typed into a link-length text box: clamp it (zero is allowed)
% and rebuild.
S = getappdata(fig,'dhstate');
if ~isvalid(fig), return; end
v  = str2double(get(S.lengthEdit(i),'String'));
sl = S.lengthSliders(i);
if isnan(v)
    dhSyncEdits(fig);
    return;
end
v = min(max(v, sl.Min), sl.Max);
sl.Value = v;
S.lastLen(i) = v;
S.tLastRebuild = tic;
setappdata(fig,'dhstate',S);
dhRebuild(fig);
dhSyncEdits(fig);
end


function dhNewRobot(fig)
% Ask for another robot definition and build it in place of this window.
% Closing the form without changing anything keeps the current robot as it is.
if ~isvalid(fig), return; end
S = getappdata(fig,'dhstate');
if isempty(S), return; end

[~, L] = dhReadQL(fig);                 % current lengths, in metres

% The form opens PRE-FILLED with what is on screen now: changing one joint out
% of four should not mean retyping the whole robot.
cfg = dhInputDialog(S.jointSeq, L, S.jointAlpha*180/pi, S.baseOffsets);

% Nothing changed (accepted as-is, or the form was closed): keep this window.
tw = cfg.twist_deg(:)';
if isempty(tw), tw = zeros(1,numel(cfg.jointSeq)); end
same = strcmp(cfg.jointSeq, S.jointSeq) && ...
       isequal(cfg.linkLengths(:)', L) && ...
       isequal(tw, S.jointAlpha(:)') && ...
       isequal(cfg.baseOffsets(:)', S.baseOffsets(:)');
if same
    return;
end

% Stop the old timer before its figure goes away, drop the matching home
% figure, and build the new robot exactly as a fresh start would.
if isfield(S,'timer') && isa(S.timer,'timer') && isvalid(S.timer)
    stop(S.timer); delete(S.timer);
end
homeFig = findall(0,'Type','figure','Name','Robot - home configuration');
if ~isempty(homeFig), delete(homeFig); end
delete(fig);

dhLaunchApp(cfg);
end


function dhClose(fig)
% Stop the timer and close the window cleanly.
S = getappdata(fig,'dhstate');
if ~isempty(S) && isfield(S,'timer') && isa(S.timer,'timer') && isvalid(S.timer)
    stop(S.timer); delete(S.timer);
end
delete(fig);
end
