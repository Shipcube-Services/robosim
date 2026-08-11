# Running the MuJoCo Viewer with Meta Quest VR Teleop

Step-by-step instructions for connecting a Quest headset, launching the
helper app on it, and starting the MuJoCo viewer under VR control. For the
one-time environment install (`uv sync`, why `mjpython` is required, etc.)
see the main [README](../README.md) first if you haven't already.

## 1. Connect the headset

1. Plug the Quest into your Mac with a USB-C **data** cable (some
   charge-only cables won't work).
2. Put the headset on. A permission dialog should appear:
   **"Allow USB Debugging?"** — check **"Always allow from this computer"**
   and select **Allow**.
   - This dialog only requires Developer Mode to already be enabled
     (Meta Horizon mobile app → your device → **Headset Settings** →
     **Developer Mode**). If you don't see the prompt at all, Developer
     Mode is probably off.
3. Verify the connection from your Mac:

   ```bash
   adb devices
   ```

   Expected:
   ```
   List of devices attached
   340YXXXXXXXXXX    device
   ```
   - Shows `unauthorized` instead of `device`: take the headset off and
     back on to re-trigger the permission prompt.
   - Shows nothing: try a different cable/port.

## 2. Launch the MuJoCo viewer

```bash
uv run mjpython -m robosim.vr.teleop
```

(`mjpython`, not plain `python`/`uv run robosim-vr-teleop` — macOS requires
MuJoCo's interactive viewer to run on `mjpython`'s main thread.)

On the **first** run this also installs the helper app to the headset —
you should see `APK installed successfully.` printed. This can take up to
~15-20 seconds over USB.

Optional flags:
- `--ip <address>` — connect over Wi-Fi instead of USB (see the README for
  finding the headset's IP).
- `--mock` — skip the headset entirely and drive the arms with a synthetic
  test motion, useful for confirming the sim itself works.
- `--scale <n>` — clutch sensitivity multiplier (default `1.0`).

## 3. Open the helper app on the headset

It should launch automatically. If the headset was showing its "put the
headset on" screen when the script started, put it on now — Quest hands
focus to the newly-launched app once it detects you're wearing it.

If it still doesn't come to the foreground, launch it manually:

```bash
adb shell am start -n "com.rail.oculus.teleop/com.rail.oculus.teleop.MainActivity" \
  -a android.intent.action.MAIN -c android.intent.category.LAUNCHER
```

**What you'll see in the headset is just a handful of coloured
blocks — that's expected.** This helper app (`oculus_reader`'s
`teleop-debug` APK) is a bare-bones data logger, not a real VR
application; those blocks are its debug visualization of tracked
controllers, and its 3D controller models are the old Quest 2 Touch
design (the app predates Quest 3 and nobody's updated its assets). None
of that affects the actual tracking data.

The screen that matters is the **MuJoCo viewer window on your Mac**.

## 4. Controls

- **Grip** (either controller) — hold to engage that arm's clutch: the
  arm follows your hand's motion relative to where you squeezed. Release
  to freeze the arm in place and reposition your hand freely.
- **Index trigger** — opens/closes that hand's gripper.

## 5. Verifying the data feed directly

To confirm the headset is streaming real pose/button data independent of
the simulator, in a separate terminal:

```bash
adb logcat -s wE9ryARX
```

You should see a continuous stream of lines like:
```
l:<16 numbers> |r:<16 numbers> &L,leftJS 0.0 0.0,leftTrig 0.0,leftGrip 0.0,R,...
```

- If **`l:` and `r:` are byte-for-byte identical** across multiple lines,
  only one physical controller is actually being tracked (its pose is
  being mirrored into both slots) — check that both controllers are
  powered on (give them a shake; they sleep when idle).
- If nothing appears at all, the app isn't running — see step 3.

## Known issues / things to expect

- **Quest 3/3S caveat:** the helper app was built for Quest 2 and was
  never officially updated for Quest 3 hardware. In testing here it
  tracked a Quest 3S's controllers correctly regardless, but if yours
  doesn't get detected at all, see the fork linked in the README.
- **Motion direction may feel off on first try.** The controller→world
  axis mapping (`src/robosim/vr/quest_reader.py`, `_AXIS_REMAP`) is a
  best-effort default; if the arm moves in an unexpected direction
  relative to your hand, that's the one place to adjust.
- **Clicking Reset in the MuJoCo viewer is safe.** Earlier versions of
  this code could produce erratic/looping motion after a Reset (a data
  race between the GUI's reset and the teleop control loop, both touching
  the same simulation state on different threads); this is fixed as of
  the current code — Reset now cleanly re-anchors both arms to their
  reset pose instead.
