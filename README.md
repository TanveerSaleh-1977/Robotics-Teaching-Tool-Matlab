# Denavit–Hartenberg Frames — Dynamic Robot Generator

One self-contained MATLAB script that teaches standard (distal)
Denavit–Hartenberg frame assignment for **2-, 3- and 4-DOF** robots, draws the
links **physically**, supports **planar and spatial** motion, and updates the
numeric **HTM and end-frame coordinates live** — with **degree sliders**, an
**editable text box per row**, **continuous animation** while you drag, and the
result shown in **symbolic (general) form** beside its numeric value.

You no longer edit arrays at the top of the file: a **pop-up form** asks for the
robot as soon as you start the script.

## Files

| File | Purpose |
|------|---------|
| `DH_Robot_LiveScript.m` | The whole thing — everything is in this one script. |
| `standalone\installer\DHRobot_Setup.exe` | **The shareable Windows installer**: it installs the app *and* the free MATLAB Runtime, so a machine without MATLAB can run it. |
| `standalone\README.txt` | What to tell the person you send the installer to (plus how to rebuild / go offline). |
| `standalone\build\DHRobot.exe` | The compiled application itself (needs the MATLAB Runtime 24.2 already installed). |
| `standalone\build_exe.m` | Rebuilds both of the above (`build_exe` from the `standalone` folder). |
| `preview_input_form.png` | The pop-up form that collects the robot definition. |
| `preview_UI_full.png` | The live control window (sliders + text boxes + symbolic matrices beside their numeric values, and the **New robot** button in the strip). |
| `preview_UI_scara.png` | A 4-DOF **SCARA** (`RRPR`, `d1 = 0.50 m`): the raised base is drawn as a **column**, the arm sits at shoulder height, and `q_3 = −40 mm` lifts the wrist *down* (`z = d1 − 0.04`). |
| `preview_UI_RP_polar.png` | A 2-DOF **polar/telescopic** arm (`RP`, `α1 = 90°`): swing + extension along the boom (`q_2 = 450 mm`), the slide's travel drawn as the boom. |
| `preview_UI_zoom.png` | The same window zoomed hard into the plot — the drawing stays inside the plot band. |
| `preview_UI_4dof.png` | A 4-DOF arm with a zero-length link. |
| `preview_UI_XYview.png` | The XY button pressed: orthographic plan view, with the four view buttons in the strip above the readout. |
| `preview_planar_RRR.png` | Planar 3-DOF arm. |
| `preview_spatial_RRR.png` | Spatial 3-DOF arm (base twisted 90°). |
| `preview_prismatic_RPR.png` | RPR (with a prismatic slide). |
| `README.md` | This note. |

## Requirements

* MATLAB **R2020b or newer** (developed and tested on **R2024b**).
* **Robotics System Toolbox** (checked in section 0).
* No Symbolic Math Toolbox: the general (symbolic) HTMs are produced by a small
  term algebra inside the script. That is also what lets the same code be
  compiled into a standalone `.exe` — the Symbolic Math Toolbox cannot be
  deployed.

## How to test on your computer

1. Open `DH_Robot_LiveScript.m` in MATLAB and press **Run** (F5).
2. A **pop-up form** appears. Fill it in and press **Build**
   (press **Use defaults** to skip it and use the defaults in section 1):

   | Field | Meaning | Example |
   |-------|---------|---------|
   | Joint sequence | one letter per joint (`R` = revolute, `P` = prismatic), 2–4 letters | `RRR` |
   | Link lengths | metres, one per joint (0 is allowed) | `0.40 0.30 0.20` |
   | Joint twists alpha | degrees, optional; `90` on a joint makes the arm spatial | `90 0 0` |
   | Base offsets d | metres, optional | `0.10 0 0` |
   | Make it spatial | tick to twist joint 1 by 90° for you | — |

   The form validates as you press **Build**: bad input shows a red message and
   the form stays open, so you can correct it.
3. Read the Command Window: DH table, every symbolic HTM, robot summary,
   numeric cross-check, and the **planar/spatial verdict**.
4. In the *live control* window each joint and each link has a row:

   ```
   [ label ]  [ slider ]  [ text box ]
   ```

   * **Drag a slider** → the robot animates continuously while you drag
     (a timer watches the sliders, so it keeps up with the motion).
   * **Type a number in a text box and press Enter** → the value is clamped
     and the robot animates smoothly to it. Invalid input is ignored.

### Units in the UI

| Row | Unit | Range |
|-----|------|-------|
| Revolute joint `q_i` | **degrees** | −180 … 180 |
| Prismatic joint `q_i` | **millimetres** | −stroke … +stroke (both ways) |
| Link length `a_i` | **millimetres** | 0 … 1000 (zero is allowed) |
| Stroke `s_i` (a `P` row) | **millimetres** | 0 … 1000 (travel each way) |

A **prismatic joint travels both ways** from its zero, so a SCARA lift can go
**down** (`q_3 = −40 mm` gives `z = d1 − 0.04`) as well as up; a cylindrical
column can rise from its base. The `P` row in the *Link lengths* block is the
**stroke** `s_i`, i.e. the travel each way, and the DH link length `a_i` stays
0. A base offset on a `P` joint is refused, because for a slide `d_i` **is**
the joint variable and there is no room for a separate constant.

(Internally everything is converted to radians/metres for the toolbox.)

A **link length of zero** is accepted: two joint origins simply coincide and
the arm drops that segment (type `0.40 0 0.20` in the form). A prismatic
joint whose stroke is zero becomes a fixed joint (its slider is effectively
locked). At least one link length must be positive.

### A raised base is drawn as a column

`d1` is not a link: it is the pedestal along joint 1's axis. The window draws
it that way - a thicker vertical column from the floor up to the shoulder, with
the first link starting at the **top** of the column. A SCARA therefore looks
like a SCARA (column, then the two arm links at shoulder height) instead of a
diagonal strut from the floor. The same split applies to later joints: a
`d_i` offset is drawn as a pedestal between parallel axes, `a_i` as the link.

### Defaults (section 1 of the script)

The form is pre-filled with these; they are also used if you press
*Use defaults* or close the form:

```matlab
defaultJointSeq    = 'RRR';            % 2-4 letters: R = revolute, P = prismatic
defaultLinkLengths = [0.40 0.30 0.20]; % metres, one value per joint
defaultTwist_deg   = [];               % DH twist alpha_i in DEGREES ([] = all zero)
defaultBaseOffsets = [];               % optional d_i (m); [] = all zeros
useInputDialog     = true;             % false -> skip the form entirely
```

Set `useInputDialog = false` if you want to run without the form (handy for
scripting or head-less testing).

## Planar vs spatial

A robot is **planar** when every DH twist `alpha_i` is a multiple of 180°, so
all joint axes are parallel. Give a joint a 90° twist and the arm leaves the
plane — it becomes **spatial** (3-D). In the form, type the twists (e.g.
`90 0 0`) or tick **Make it spatial**.

The script prints `... -> PLANAR robot.` or `... -> SPATIAL robot.`, and the
plot title shows `(planar)` / `(spatial)`.

## Live readout (scrolling pane below the plot)

The pane is a table of paired lines — **the symbolic matrix on the left, its
numeric value on the right (2 decimal places)**:

```
MATRIX (symbolic general form)                    | NUMERIC (2 d.p.)
shorthand:  C1 = cos(theta1)   C2 = cos(theta2)   C3 = cos(theta3)   C12 = cos(theta1 + theta2)   C123 = cos(theta1 + theta2 + theta3)   S1 = sin(theta1)
            S2 = sin(theta2)   S3 = sin(theta3)   S12 = sin(theta1 + theta2)   S123 = sin(theta1 + theta2 + theta3)
T(0 -> 1)   [joint 1, theta1]
[ C1, -S1, 0, C1*a1 ]                             | [    1.00,    0.00,    0.00,    0.40 ]
[ S1,  C1, 0, S1*a1 ]                             | [    0.00,    1.00,    0.00,    0.00 ]
[  0,   0, 1,     0 ]                             | [    0.00,    0.00,    1.00,    0.00 ]
...
T(0 -> 3)   (composed, simplified)
[ C123, -S123, 0, C1*a1 + C12*a2 + C123*a3 ]      | [    0.64,   -0.77,    0.00,    0.76 ]
[ S123,  C123, 0, S1*a1 + S12*a2 + S123*a3 ]      | [    0.60,    0.80,    0.00,    0.56 ]
```

* Angle sums are written with the conventional kinematics shorthand
  `C1 = cos(theta1)`, `C12 = cos(theta1 + theta2)`, `C123 = cos(theta1 + theta2 + theta3)`
  and likewise `S1`, `S12`, `S123`. This is what keeps the long rows readable.
  The `shorthand:` line above the table spells out exactly the abbreviations that
  occur, in the window **and** in the Command Window.
* The shorthand is **display only**: the same matrix is also kept as
  coefficient/angle-sum terms, and those terms feed the numeric column and the
  section-5 cross-check with the toolbox — nothing is approximated.
* Every `T(i-1 -> i)` and the composed `T(0 -> N)` is shown in **general form**:
  the link lengths appear as `a1, a2, …`, the twists as `alpha1, alpha2, …`
  and the offsets as `d1, d2, …` **only where they are actually needed**
  (a value that is zero is not shown), and the moving variables stay as
  `theta1, theta2, …` (or `d1, d2, …` for prismatic joints). So a 3-link planar
  arm reads exactly like the textbook `a1, a2, a3` / `theta1, theta2, theta3`.
* The numeric column beside `T(i-1 -> i)` is that **local** matrix
  (`inv(T(0 -> i-1)) * T(0 -> i)`), not `T(0 -> i)`, so both columns describe
  the same transform. A prismatic joint keeps its twist: with `alpha2 = 90°`
  the slide's frame is turned, and that rotation shows in the composed matrix
  (a twisted slide is not a planar arm).
* The composed `T(0 -> N)` is fully simplified, so it collapses to the familiar
  closed form (`C123`, `S123`, `C1*C23`, …). Sums may skip a prismatic joint: in
  an RPR arm there is no `theta2`, so the arm angle is `C13 = cos(theta1 + theta3)`.
  Angle sums are only merged where the joints really are coplanar; across a 90°
  twist the factors stay separate, e.g. a spatial 3R reads `C1*C23`.
  Long rows are soft-wrapped and their numeric value is aligned on the
  following line.
* The general form follows the **link-length sliders** too. Sliding `a1` down to
  zero removes `a1` from `T(0 -> 1)` and from the composed matrix (the column
  shows `0`, and a spatial 3R becomes `C1*(C2*a2 + C23*a3)`); growing it again
  brings `a1*C1` back. Only the crossing of zero costs anything, so dragging
  stays smooth.
* Above the matrices: the live joint values `q` (deg / mm), the end position
  `(x, y, z)` in metres and the origin of every frame `J0 … JN`.

The end position is marked with a red dot and — instead of a boxed coordinate
label — with **dashed projections of its x, y and z components onto the three
axes**: one drop to the XY plane, then one line to the foot on the X axis and
one to the foot on the Y axis, each foot dotted in the colour of that axis
(x red, y green, z blue) and annotated `x = 0.76 m`, `y = 0.36 m`,
`z = 0.00 m`. A planar arm therefore shows a flat L in the XY plane with
`z = 0.00 m`, a spatial arm shows the vertical drop as well. The coordinate
frame of every joint is redrawn as thick coloured axes on top of the robot, so
the frames stay visible. The axes title carries the end position, and the
window keeps the classic toolbar, so you can rotate/zoom the 3-D view.

### One-click planar views (XY, YZ, ZX) and back

Four buttons sit in a row in the free strip under the sliders, just above the
readout:

| button | camera | you see |
|--------|--------|---------|
| **XY** | looking down −z | x right, y up — the plan view |
| **YZ** | looking along −x | y right, z up |
| **ZX** | looking along −y | x right, z up |
| **3D** | the usual perspective view | the arm in space |

Each plane button switches to an **orthographic** camera, so the drawing is a
true projection of that plane — with the default perspective camera the far end
of the arm would be drawn smaller and the “planar” view would not measure
correctly. **3D** takes you straight back to the perspective view; the button
of the view currently on screen is highlighted, so it is always clear where you
are. The buttons are ordinary controls, so they stay in that strip whatever the
camera does — text drawn inside the axes would have been anchored in the data
box and would slide away with the view (which is exactly what used to hide the
way back). Rotating with the toolbar afterwards keeps working — the camera
simply starts from the plane you chose.

### Changing to another robot (New robot)

The **New robot** button sits at the right end of that same strip. It opens the
definition form again, **pre-filled with the robot that is on screen** — change
the joint sequence, a length, a twist or an offset and press *Build*; the old
window (and its home-configuration figure) is replaced by the new robot in the
**same MATLAB session**. Nothing has to be restarted, and closing the form with
no changes keeps the window exactly as it is.

That is also why the build lives in `dhLaunchApp(cfg)`: the script calls it once,
and the button calls it again. The `P`/`R` sequence, the joints, the symbolic
matrices, the toolbox chain and the readout are rebuilt from scratch, so a
4-DOF SCARA can follow a 3-DOF cylindrical arm without stale state.

### The plot area fills the window

Two things decide how large the arm is drawn, and both are now used in full:

* **the height of the plot band.** In a 3-D axes MATLAB scales the plot box to
  the *shorter* side of the axes rectangle, so a wide, short band wastes its
  width — the box fits the height and leaves wide empty margins on either side.
  The band now runs from just above the readout to the top of the window; the
  slider column ends higher up on the left, so the two do not compete for the
  same space.
* **the height of the plot box itself.** A cube is the right box for a
  *spatial* arm, but a **planar** arm (every DH twist a multiple of 180°)
  keeps every joint in a single horizontal plane: its z does not depend on the
  revolute angles at all, it is only the sum of the fixed offsets and the
  prismatic strokes. Drawing a cube for such an arm therefore spends most of
  the axes height above and below a flat arm. The plot box is a thin slab
  instead. `dhPlotZRange` evaluates the chain at the ends of every prismatic
  stroke (at most 2⁴ = 16 configurations) and takes the vertical extent of
  every joint origin, so a raised base, a vertical stroke or a plane sitting
  above z = 0 all stay inside the box. A spatial arm keeps its cube.

Together the two changes make the arm about **twice as large** — measured on a
planar 3-DOF arm at the default window size: 93 → 199 pixels per metre — and
the workspace now fills the width of the band instead of floating in the middle
of it.

### The plot stays inside its own band

The plot has its own white band at the top right of the window, with a clear
margin above the readout and to the right of the slider column, so the axis
labels and tick labels never touch the controls. On top of that:

* every drawing object of the **live** window is created with `Clipping = 'on'`,
  so **zooming in can never spill the robot or its labels over the sliders or
  the readout**;
* the live window does **not** call `show()` on every frame (see below), so the
  small orientation triad that `show()` adds as a separate axes — the thing
  that used to sit on top of the slider column — simply does not exist there;
* the projection lines and their labels are ordinary axes objects with
  `Clipping = 'on'`, and the axes limits are only re-fitted when the link
  lengths change, so a zoom can never push them over the controls.

The static *home configuration* figure (section 6) does use `show()`, and there
the triad is moved out of the figure corner into the plot area.

### Fast, continuous animation

Creating a graphics object costs far more than moving one, and `show()` has to
re-create the robot's whole mesh on every call — measured at **~170 ms per
frame** (about 6 frames per second), which is what made the sliders feel laggy
and jerky.

The live window therefore creates its scene **once** and only writes coordinates
afterwards:

* one patch holding all link cylinders, one for the slide rails, one for the
  joint spheres, one NaN-separated line per frame axis, plus the end marker and
  the three projection lines — **no graphics object is created or deleted while
  you drag**;
* the link/joint geometry templates are recomputed (not re-created) when a link
  length changes, so dragging a length slider is smooth too;
* the projection *values* (three small text objects) are re-written with the
  readout, about 10 times a second, not on every animation frame;
* the forward kinematics still comes from the `rigidBodyTree`, and `show()`
  still draws the home-configuration figure;
* the two text-heavy parts — the plot title and the 30-line readout pane — are
  refreshed at ~10 Hz instead of every frame, because repainting that pane is
  itself the most expensive single thing in a frame.

Measured on R2024b in a head-less run (software renderer, so a real display is
faster): **~22–30 ms per frame** while dragging a 3-DOF arm (**~28 ms** for
4-DOF), down from ~650 ms — comfortably faster than the 25 Hz at which the
update timer polls the sliders. The taller plot band costs about 20 % more per
frame than the old short band in that software renderer, which repaints every
pixel; a real GPU does not pay that, and it is a good trade for an arm drawn
twice as large.

## Examples to try (type these in the form)

| Joints | Link lengths | extra | Result |
|--------|--------------|-------|--------|
| `RRR` | `0.40 0.30 0.20` | — | planar 3R |
| `RRR` | `0.40 0.30 0.20` | tick *Make it spatial* (or twists `90 0 0`) | spatial 3R |
| `RPR` | `0.40 0.25 0.30` | — | revolute-prismatic-revolute |
| `RRR` | `0.40 0 0.20` | — | zero-length middle link |
| `RRRR` | `0.30 0.30 0.25 0.15` | — | 4-DOF |
| `RRR` | `0.40 0.30 0.20` | base offsets `0.10 0 0` | raised base: a column, then the arm |
| `RPP` | `0 0.50 0.60` | twists `0 90 0`, offsets `0.30 0 0` | **cylindrical**: rotate, lift (`d2`), reach (`d3`) |
| `RRPR` | `0.35 0.25 0.20 0` | twists `0 0 0 0`, offsets `0.50 0 0 0` | **SCARA**: shoulder at 0.50, lift down with `q3 < 0`, wrist `q4` |
| `RP` | `0 0.60` | twists `90 0` (or tick *spatial*), offsets `0.50 0` | **polar / telescopic**: the base swings, `q2` is the arm length along the boom (`0.60` = stroke = max reach) |

## Standalone Windows app (for people who do not have MATLAB)

`standalone\DHRobot_Setup.exe` is a normal Windows installer (2.6 MB). Run it on
any 64-bit Windows machine and it installs the app **and** the free
**MATLAB Runtime 24.2** (about 1.5 GB, downloaded once by the installer), then
puts a *DH Robot - Frames and HTMs* shortcut in the Start menu. No MATLAB, no
licence, no toolbox is needed on that machine.

The app is the same code: `build_exe.m` turns the script into a function
(`dh_robot_app.m`), compiles it with `mcc`, and packs the installer. Two small
differences from the MATLAB version:

* a compiled app has no Command Window, so the report that the script prints
  (DH table, every HTM, shorthand legend, robot summary, cross-check) opens in
  its own scrollable window;
* the toolbox check is skipped, because the toolbox ships inside the runtime.

Rebuild after editing the script:

```matlab
cd standalone
build_exe            % convert + compile + pack (a few minutes)
build_exe('convert') % only regenerate dh_robot_app.m
```

If the machine that builds it has no internet, or the target machine must stay
offline, pass `'RuntimeDelivery','installer'` in `build_exe.m` and point
`compiler.package.installer` at a downloaded `MCRInstaller.exe`
(`compiler.runtime.download` fetches it); the installer then carries the whole
runtime (~4 GB).

## How the requirements are met

1. **Dynamic DH table & configuration** — section 1 (pop-up form + validation)
   and section 2. `dhGenerate` builds `alpha`, `a`, `d`, `theta` from the joint
   sequence, link lengths, twists and offsets. The form can be reopened from the
   live window (**New robot**), pre-filled with the current robot, so several
   robots can be compared in one session.
2. **Math & HTMs** — section 3. Each `T(i-1 -> i)` is symbolic (standard DH
   matrix shown as a LaTeX equation), printed step by step, then the composed
   forward-kinematics product — all in general form, with joint-angle sums
   abbreviated to the textbook `C12` / `S123` notation (the legend is printed
   next to them). The symbolic layer is a small term algebra inside the script
   (coefficient × cos/sin of an angle sum × a_i/d_i), so it needs no Symbolic
   Math Toolbox and can be compiled.
3. **Robot construction** — section 4. `dhBuildRobot` loops over the sequence,
   one `rigidBody` per letter, joint type from the letter, DH applied with
   `setFixedTransform(...,'dh')`, plus physical geometry (link cylinder,
   pedestal for a `d` offset, joint sphere, prismatic rail).
4. **Interactivity** — section 7. Degree/mm sliders + editable text boxes,
   with continuous animation while dragging; slides travel both ways.
5. **Visualization** — sections 6 and 7. `show(robot, q, 'Frames','on')`
   renders the static home configuration with the XYZ frame at every joint and
   the physical links; the live window draws the same physical links (cylinder
   per link, a column per `d` offset, sphere per joint, rail per prismatic
   joint, thick XYZ axes at every joint origin, the end marker with the dashed
   x/y/z projections of the end position onto the axes, and the
   symbolic-beside-numeric readout) with objects created once and updated in
   place, so it animates continuously.
6. **Several robots, one session** — the whole build is `dhLaunchApp(cfg)`;
   the script calls it once and the **New robot** button calls it again after
   re-opening the form. The previous window, its timer and its home figure are
   replaced, so no restart is needed.

## Validation

Section 5 substitutes a test configuration into the general symbolic matrix and
compares it with the toolbox forward kinematics — agreement is at machine
precision. The UI was additionally driven head-lessly (the pop-up form
included, by filling it and pressing *Build*): the form rejects a bad sequence,
shows the message, stays open and accepts the correction; continuous drag
(0 → 90°), typed values, invalid input, zero link lengths, a zero prismatic
stroke, base offsets, and a spatial arm all behave correctly; the symbolic
matrices keep `a1`, `alpha1` (spatial) and `d1` (offsets) exactly where
expected, with every `cos`/`sin` of a joint-angle sum abbreviated to
`C…`/`S…` (checked on `RRR`, `RRRR`, `RPR` and a spatial `RRPR` chain: no
`cos(theta…)` is left in the displayed matrices, and the shorthand does not
change the numeric result). The end-position projections were checked the same
way: the dashed feet land exactly on (x, 0, 0) and (0, y, 0) for the current
pose, and the three value labels are re-written when the slider is released.
The thin plot box was checked the same way: for a planar `RRR` chain, a planar
chain with 180° twists, an `RPR` with a vertical stroke and an `RRP` with a
raised base, 300 random reachable configurations all fell inside the slab
(and a spatial arm keeps the cube, as before).

## Notes

* The window is a classic `figure` (robotics `show` does not accept a UI
  axes), so the sliders and text boxes are classic `uicontrols` and a timer
  provides the continuous animation.
* The live window draws its own link geometry (created once) rather than
  re-calling `show()` per frame: `show()` re-creates the robot's meshes every
  call, which costs ~170 ms per frame and is what made dragging feel laggy.
* Link thickness scales with the smallest link length so it looks right at
  any arm size.
* To use this as a Live Script: open the `.m` file and choose
  *Save As → MATLAB Live Script (\*.mlx)*.
