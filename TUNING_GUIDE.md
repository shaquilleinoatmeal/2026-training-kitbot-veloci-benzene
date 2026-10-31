# Drivetrain Tuning Guide (for freshies)

This guide walks you through tuning the kitbot drivetrain: making sure every motor and sensor
points the right way, making teleop feel good, and getting the **"Drive 10 ft"** auton to stop
on the line, straight, every time.

Go in order. Each step depends on the ones before it. If a step doesn't work, **stop and fix it**
before moving on, because later steps will be wrong too.

---

## 0. Rules (read these first)

1. **Only work on the `DriveTrain_Tuning` branch.** Never commit to `main`, `DriveTrain`, or any
   other branch. If `git status` doesn't say `On branch DriveTrain_Tuning`, stop and switch.
2. **Only change numbers in [`DriveConstants.java`](2026-training-kitbot/src/main/java/frc/robot/DriveConstants.java).**
   Everything you need to tune lives there or on the dashboard. If you think another file needs to
   change, write it down in the tuning log and ask a lead.
3. **Change one thing at a time.** Then test. Then write down what happened.
4. **Commit after every step that works**, with a message saying what you changed and why.
5. **Safety:**
   - First test of anything new: **robot on blocks** (wheels off the ground).
   - Someone always has a hand near the **space bar** (disables the robot in Driver Station).
     Enter = emergency stop.
   - Clear the area in front of and behind the robot before enabling. Yell "ENABLING".
   - Use a charged battery (above 12.5 V when resting). A dead battery makes all tuning lie to you.

---

## 1. Getting set up

### Get the code

```bash
git clone https://github.com/shaquilleinoatmeal/2026-training-kitbot.git
```

```bash
cd 2026-training-kitbot
```

```bash
git checkout DriveTrain_Tuning
```

```bash
git pull
```

Open the folder in **WPILib VS Code**. The robot project is in the `2026-training-kitbot/` subfolder.

### Every time you sit down to work

```bash
git checkout DriveTrain_Tuning
```

```bash
git pull
```

Someone else may have pushed changes since you last worked. Always pull first.

### Try it in simulation first (no robot needed)

From the `2026-training-kitbot/` folder:

```bash
./gradlew test
```

This runs a simulated "Drive 10 ft" auton and checks it lands within 2 inches. If this fails
after you changed something, your change broke the auton — undo it before going to the robot.

```bash
./gradlew simulateJava
```

This opens the robot simulator. You can drive with a controller and run auton without a robot.
Simulation is a good place to see what a number *does*, but the real robot always has the final say.

### Deploy to the real robot

Connect to the robot (USB, ethernet, or the robot's Wi‑Fi), then in VS Code:
**Ctrl+Shift+P → "WPILib: Deploy Robot Code"**.

### Open the dashboard

Use **Elastic** or **Shuffleboard** (both come with WPILib). The values you'll care about:

| Folder | What's in it |
| --- | --- |
| `Tuning/` | Numbers **you** can type into to change robot behavior live |
| `Drive/` | What the sensors read: left/right inches, speeds, heading |
| `Auto/` | What the auton is trying to do vs. what it's actually doing |
| `Auto` (chooser) | Pick which auton runs |
| `Reset sensors` (button) | Zeroes encoders, gyro, and field position (works while disabled) |
| `Field` | Where the robot thinks it is on the field |

> **Important:** Values typed into `Tuning/` are **temporary**. They reset to the numbers in
> `DriveConstants.java` when the robot reboots or code is redeployed. When you find a good value,
> **copy it into `DriveConstants.java` and commit it** (see Step 6).

---

## 2. Direction checks (robot on blocks)

Nothing else works if a motor or sensor is backwards. Do all of these with the robot **on blocks**
in **teleop**.

Controls: **left stick up/down** = forward/back, **right stick left/right** = turn,
**hold right bumper** = full speed (otherwise capped).

| Check | How | Expected | If wrong, change in `DriveConstants` |
| --- | --- | --- | --- |
| Right side spins the right way | Push left stick forward | Both sides' wheels spin "forward" | `RIGHT_SIDE_INVERTED` (flip true/false). If the **left** side is backwards, ask a lead. |
| Motors on the same side don't fight | Push forward, listen/feel | Smooth, no grinding or buzzing, wheels spin freely | `FOLLOWER_OPPOSES_LEADER` — **disable right away** if you hear grinding |
| Turning goes the right way | Push right stick right | Robot would turn right (left wheels forward, right wheels back) | `INVERT_TURN` |
| Left encoder | Push forward, watch `Drive/Left inches` | Goes **up** | `LEFT_ENCODER_REVERSED` |
| Right encoder | Push forward, watch `Drive/Right inches` | Goes **up** | `RIGHT_ENCODER_REVERSED` |
| Gyro | Disable robot. Turn it by hand to the **left** (counter‑clockwise from above). Watch `Drive/Heading deg` | Goes **up** | `GYRO_REVERSED` |

If an encoder reads **0 the whole time**, it's probably unplugged or in the wrong DIO port
(`LEFT_ENCODER_PORTS` / `RIGHT_ENCODER_PORTS`). Check the wiring with a lead.

✅ Commit: `git commit -am "Direction checks: flipped X"` (or "all directions already correct").

---

## 3. Encoder calibration (make "inches" mean inches)

1. Put the robot on the floor, disabled. Mark where the front bumper is with tape.
2. Click **Reset sensors** on the dashboard. `Drive/Average inches` should read 0.
3. **Push the robot by hand exactly 120 inches (10 ft) straight forward.** Use a tape measure.
4. Read `Drive/Average inches`. Call it `reading`.
5. New calibration:

   ```
   new CALIBRATION_FACTOR = old CALIBRATION_FACTOR × (120 ÷ reading)
   ```

   Example: old = 1.0, reading = 112 → new = 1.0 × 120 / 112 = **1.071**
6. Put the new value into `CALIBRATION_FACTOR`, deploy, and repeat. Done when the reading is
   within **±1 inch** of 120.

Also double check `WHEEL_DIAMETER_INCHES` (6.0) actually matches the wheels.

✅ Commit: `git commit -am "Encoder calibration: CALIBRATION_FACTOR = 1.071"`.

---

## 4. Teleop feel (driver comfort)

These only affect driving with the controller. Get a driver's opinion.

| What it feels like | Knob | Where | Try |
| --- | --- | --- | --- |
| Too fast / too slow normally | `Tuning/Teleop max output` | Dashboard (live) | 0.4 – 0.8. Default 0.6. Right bumper always gives 1.0. |
| Jerky / tips when starting or stopping | `TELEOP_RAMP_SECONDS` | `DriveConstants` | Bigger = smoother but laggier. Try 0.1 – 0.3 |
| Robot creeps when sticks are let go | `DEADBAND` | `DriveConstants` | Raise a little (0.08 → 0.1) |
| Small stick moves do nothing | `DEADBAND` | `DriveConstants` | Lower a little (0.08 → 0.05) |

Don't touch `CURRENT_LIMIT_AMPS` or `VOLTAGE_COMPENSATION_VOLTS` without a lead.

✅ Commit when the driver is happy.

---

## 5. Auton tuning ("Drive 10 ft")

### How the auton works (the short version)

The auton drives 120 inches. It uses three ideas together:

- **Motion profile (trapezoid):** Instead of slamming to full speed, it plans a smooth path —
  speed up at `max accel`, cruise at `max velocity`, slow down, stop. The `Auto/Setpoint` values
  show where the plan *wants* the robot to be right now.
- **Feedforward (kS, kV, kA):** A guess at how much power is needed to follow that plan.
  kS = power just to get moving (beat friction), kV = power per unit of speed,
  kA = power per unit of acceleration. A good feedforward does ~90% of the work.
- **PID (kP, kD):** Fixes the leftover error. kP pushes harder the farther behind the robot is.
  kD calms things down if it's overshooting or wobbling.
- **Heading kP:** Steers to keep the robot pointing straight, using the gyro.

Auton ends when it's within `Auto tolerance inches` of 120 and nearly stopped, or after
`AUTO_TIMEOUT_SECONDS` (6 s), whichever is first.

### How to run a test

1. Robot on the floor, tape mark at the starting line and at 10 ft.
2. Disabled → click **Reset sensors** → pick **Drive 10 ft** in the `Auto` chooser.
3. Driver Station → **Autonomous** → Enable. Hand on space bar.
4. After it stops, write down:
   - Where it actually stopped (tape measure from start, inches)
   - `Drive/Average inches` (what the robot thinks)
   - `Drive/Heading deg` at the end (should be ~0)
   - Did it overshoot and come back? Wobble? Stall short? Time out?
5. Change **one** `Tuning/` value, Disable, re-run. The new value is picked up at the start of
   each run — no redeploy needed.

### Tuning order

Default values to start from:

| Dashboard key | Default | What it does |
| --- | --- | --- |
| `Tuning/Auto max velocity in per sec` | 50 | Top cruising speed |
| `Tuning/Auto max accel in per sec2` | 50 | How fast it speeds up / slows down |
| `Tuning/Auto distance kP` | 0.02 | Push harder when behind/ahead |
| `Tuning/Auto distance kD` | 0.002 | Damp overshoot / wobble |
| `Tuning/Auto heading kP` | 0.02 | Steer to stay straight |
| `Tuning/Auto tolerance inches` | 1.0 | How close counts as "there" |

**Step 5a — Go slow first.** Keep max velocity at 50 and max accel at 50 until it lands reliably.

**Step 5b — Distance kP.**
- Stops short / drifts in slowly → raise kP a bit (×1.5 each try: 0.02 → 0.03 → 0.045).
- Overshoots and backs up, or oscillates back and forth at the end → lower kP.

**Step 5c — Distance kD** (only if kP is good but there's still overshoot/wobble).
Raise in small steps (0.002 → 0.004). Too much kD makes the motors buzz or stutter.

**Step 5d — Heading kP.**
- Robot curves off to one side → raise heading kP (0.02 → 0.03).
- Robot snakes left/right → lower heading kP.
- Robot **spins in circles or veers hard right away** → the gyro direction is backwards.
  Go back to Step 2 and fix `GYRO_REVERSED`.

**Step 5e — Speed it up.** Once it lands within 1 inch 3 times in a row, raise max velocity and
max accel by ~10 at a time. Re-check kP after each bump. If it starts slipping wheels (you hear
squeaking, or tape-measure distance ≠ `Average inches`), lower max accel.

**Step 5f — Tolerance.** Leave at 1.0 unless the auton keeps timing out while sitting right next
to the line — then try 1.5.

### Using the graphs

In Elastic/Shuffleboard, make a line graph with **`Auto/Setpoint inches`** and
**`Auto/Traveled inches`** on the same plot.

- Lines on top of each other → great.
- Traveled always **behind** setpoint → feedforward is too weak (ask a lead about kV / SysId),
  or kP too low.
- Traveled **overshoots then comes back** → kP too high or need more kD.

✅ Commit after each good step.

---

## 6. Saving your values (don't skip this!)

Dashboard values disappear on reboot. When you've found good numbers:

1. Copy each value from the dashboard into the matching constant in `DriveConstants.java`:

   | Dashboard key | Constant |
   | --- | --- |
   | `Tuning/Teleop max output` | `TELEOP_MAX_OUTPUT` |
   | `Tuning/Auto distance kP` | `DISTANCE_KP` |
   | `Tuning/Auto distance kD` | `DISTANCE_KD` |
   | `Tuning/Auto heading kP` | `HEADING_KP` |
   | `Tuning/Auto tolerance inches` | `DISTANCE_TOLERANCE_INCHES` |
   | `Tuning/Auto max velocity in per sec` | `MAX_VELOCITY_IN_PER_SEC` |
   | `Tuning/Auto max accel in per sec2` | `MAX_ACCEL_IN_PER_SEC2` |

2. Run `./gradlew test` to make sure the sim auton still passes.
3. Deploy, reboot the robot, run the auton once more to confirm it still works.
4. Commit and push:

```bash
git add 2026-training-kitbot/src/main/java/frc/robot/DriveConstants.java TUNING_GUIDE.md
```

```bash
git commit -m "Auton tuning: kP 0.03, heading kP 0.025, max vel 70"
```

```bash
git push origin DriveTrain_Tuning
```

---

## 7. Advanced (with a lead): SysId for kS / kV / kA

The feedforward numbers (`KS_VOLTS`, `KV_VOLTS_PER_IN_PER_SEC`, `KA_VOLTS_PER_IN_PER_SEC2`) are
best measured with WPILib's **SysId** tool instead of guessed. **Do this with a lead.**

1. Robot on the floor with **at least 15 ft of clear space** in front and behind.
2. Driver Station → **Test** mode → Enable.
3. **Hold** each button until the robot gets close to the end of the space, then let go
   (letting go stops it):
   - **A** — quasistatic forward (slowly speeds up)
   - **B** — quasistatic backward
   - **X** — dynamic forward (sudden jump to 4 V)
   - **Y** — dynamic backward
4. Disable. Download the `.wpilog` file from the roboRIO (WPILib **DataLogTool**), open it in
   **SysId**, and read off kS, kV, kA (units: volts, inches).
5. Put them in `DriveConstants`, run `./gradlew test`, then redo Step 5b–5d.

---

## 8. Troubleshooting

| Problem | Likely cause | Fix |
| --- | --- | --- |
| Robot doesn't move at all in auton | Wrong auton selected, or `Do nothing` chosen | Pick **Drive 10 ft** in the `Auto` chooser |
| Auton ends early / doesn't move | Forgot to Reset sensors, or encoders backwards | Reset sensors; recheck Step 2 |
| Spins in circles in auton | Gyro direction wrong | `GYRO_REVERSED` (Step 2) |
| Runs away and never stops | An encoder is backwards or unplugged | Disable! Recheck Step 2 |
| Always stops a few inches short | kP too low or kS too low | Raise kP (Step 5b) |
| Robot stops where it thinks 120 is, but tape says otherwise | Calibration off or wheels slipping | Redo Step 3; lower max accel |
| Works on full battery, fails on low battery | Battery | Swap battery; retest |
| Dashboard value change does nothing | Changed it mid-run | Values are read at the **start** of each run — disable and re-run |
| `./gradlew test` fails | Your change broke the auton in sim | Undo your last change and try a smaller one |

---

## 9. Tuning log

Copy this table into `TUNING_LOG.md` (or a shared doc) and fill it in **every run**.
It is the most useful thing you can hand back.

| Date | Who | Battery V | What I changed | Stopped at (tape, in) | Avg inches | End heading | Notes |
| --- | --- | --- | --- | --- | --- | --- | --- |
| | | | | | | | |

---

## 10. Git cheat sheet

| I want to… | Command |
| --- | --- |
| See what branch I'm on and what changed | `git status` |
| Get everyone else's latest changes | `git pull` |
| Undo my changes to a file (not committed yet) | `git restore <file>` |
| Save my work | `git commit -am "what and why"` |
| Share my work | `git push origin DriveTrain_Tuning` |
| See recent commits | `git log --oneline -10` |

If git says something scary (merge conflict, detached HEAD, rejected push) — **stop and ask**
instead of trying random commands.
