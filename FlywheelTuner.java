package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * FLYWHEEL TUNER - find the best launcher speed and PIDF numbers.
 *
 * The launcher uses the hub's built-in velocity control (RUN_USING_ENCODER). We tell the hub a
 * target speed in TICKS PER SECOND and it adjusts the power to hold it. HOW it adjusts is set by
 * four numbers, P I D and F:
 *
 *   F (feedforward)  the power the wheel needs just to keep spinning at a speed. Get F right
 *                    first; it does most of the work.
 *   P (proportional) extra push based on how far off the speed is. More P = faster recovery
 *                    after a shot, but too much makes the speed wobble.
 *   I (integral)     slowly fixes a speed that stays a little low or high. Keep it small.
 *   D (derivative)   damps wobble. Usually 0 for a flywheel.
 *
 * GAMEPAD 1
 *   A                   flywheel ON / OFF
 *   Left / right bumper pick what to change: TARGET, P, I, D or F
 *   D-pad up / down     change the picked value by one step
 *   D-pad left / right  make the step 10x smaller / 10x bigger
 *   Y                   "Find F" test: full power for 3 seconds, then sets F, P and I for you
 *   X                   fire ONE ball, then measure how far the speed dips and how fast it recovers
 *   B                   go back to goBILDA's numbers (P 40, F 12.5, target 1250)
 *   Right trigger       intake in (to load balls)       Left trigger   intake out
 *
 * A GOOD WAY TO TUNE
 *   1. Press Y and let the Find F test finish. It gives you starting P, I and F.
 *   2. Press A to spin up. Look at "Spin-up time" and "Wobble". Wobble should be small.
 *   3. Load balls and press X. Look at "Lowest after shot" and "Recovery time".
 *   4. Raise P until recovery is quick. If wobble grows, back P off a little.
 *   5. Use TARGET to find the speed that scores from where you like to shoot.
 *   6. Copy the final numbers into the TeleOp and Autonomous OpModes.
 */
@TeleOp(name = "2 - Flywheel Tuner", group = "Test")
public class FlywheelTuner extends OpMode {

    private DcMotorEx launcher;
    private DcMotorEx intake;
    private CRServo windmillServo;
    private CRServo leftIntakeServo;
    private CRServo rightIntakeServo;

    // ---- The numbers we are tuning (start with goBILDA's) ----
    private double targetVelocity = 1250;  // ticks per second (28 ticks = 1 turn of the wheel)
    private double kP = 40;
    private double kI = 0;
    private double kD = 0;
    private double kF = 12.5;

    // Which number the d-pad changes. 0 = TARGET, 1 = P, 2 = I, 3 = D, 4 = F
    private final String[] SETTING_NAMES = { "TARGET", "P", "I", "D", "F" };
    private int selectedSetting = 0;
    private double step = 10;

    // "Close enough" to the target, in ticks per second.
    private static final double TOLERANCE = 25;

    // How long the windmill runs to push exactly one ball. Check it on the real robot: if X
    // fires two balls, make this shorter; if it fires none, make it longer. Then copy it into
    // the Autonomous.
    private static final double FEED_ONE_BALL_SECONDS = 0.20;

    private boolean flywheelOn = false;

    // ---- Spin-up measurement ----
    private final ElapsedTime spinUpTimer = new ElapsedTime();
    private boolean reachedTarget = false;
    private double spinUpSeconds = 0;
    private double highestVelocity = 0;

    // ---- Wobble measurement: lowest and highest speed seen over each 1 second ----
    private final ElapsedTime wobbleTimer = new ElapsedTime();
    private double wobbleLow = Double.MAX_VALUE;
    private double wobbleHigh = 0;
    private double lastWobbleLow = 0;
    private double lastWobbleHigh = 0;

    // ---- Shot measurement ----
    private final ElapsedTime shotTimer = new ElapsedTime();
    private boolean shotInProgress = false;
    private boolean recovered = false;
    private double lowestAfterShot = 0;
    private double recoverySeconds = 0;

    // ---- Find F test ----
    private final ElapsedTime findFTimer = new ElapsedTime();
    private boolean findingF = false;
    private double fastestSeen = 0;

    @Override
    public void init() {
        launcher         = hardwareMap.get(DcMotorEx.class, "launcher");
        intake           = hardwareMap.get(DcMotorEx.class, "intake");
        windmillServo    = hardwareMap.get(CRServo.class, "windmillServo");
        leftIntakeServo  = hardwareMap.get(CRServo.class, "left_intake_servo");
        rightIntakeServo = hardwareMap.get(CRServo.class, "right_intake_servo");

        windmillServo.setDirection(DcMotorSimple.Direction.REVERSE);
        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        applyPIDF();

        telemetry.addData("Status", "Ready. Press START, then A to spin up.");
    }

    @Override
    public void loop() {
        double velocity = launcher.getVelocity();

        if (findingF) {
            runFindFTest(velocity);
        } else {
            handleButtons();
            runFlywheel(velocity);
        }

        // The intake is always on the triggers so balls can be loaded at any time.
        double intakePower = gamepad1.right_trigger - gamepad1.left_trigger;
        intake.setPower(intakePower);
        leftIntakeServo.setPower(intakePower);
        rightIntakeServo.setPower(intakePower);

        showTelemetry(velocity);
    }

    /** Reads every button except the triggers. */
    private void handleButtons() {
        // A: flywheel on / off. Turning it on starts a new spin-up measurement.
        if (gamepad1.aWasPressed()) {
            flywheelOn = !flywheelOn;
            if (flywheelOn) {
                spinUpTimer.reset();
                reachedTarget = false;
                highestVelocity = 0;
            }
        }

        // Bumpers: pick which setting the d-pad changes.
        if (gamepad1.rightBumperWasPressed()) {
            selectedSetting = (selectedSetting + 1) % SETTING_NAMES.length;
        }
        if (gamepad1.leftBumperWasPressed()) {
            selectedSetting = (selectedSetting + SETTING_NAMES.length - 1) % SETTING_NAMES.length;
        }

        // D-pad left / right: change the step size.
        if (gamepad1.dpadRightWasPressed()) step = step * 10;
        if (gamepad1.dpadLeftWasPressed())  step = step / 10;

        // D-pad up / down: change the picked setting by one step.
        if (gamepad1.dpadUpWasPressed())   changeSelectedSetting(step);
        if (gamepad1.dpadDownWasPressed()) changeSelectedSetting(-step);

        // B: back to goBILDA's numbers.
        if (gamepad1.bWasPressed()) {
            targetVelocity = 1250;
            kP = 40;
            kI = 0;
            kD = 0;
            kF = 12.5;
            applyPIDF();
        }

        // X: fire one ball. Only if the wheel is on and up to speed, so every test is fair.
        if (gamepad1.xWasPressed() && flywheelOn && reachedTarget && !shotInProgress) {
            shotInProgress = true;
            recovered = false;
            lowestAfterShot = targetVelocity;
            shotTimer.reset();
        }

        // Y: start the Find F test.
        if (gamepad1.yWasPressed()) {
            findingF = true;
            fastestSeen = 0;
            findFTimer.reset();
            launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);  // plain power, no control
        }
    }

    /** Adds "amount" to whichever setting is picked. Nothing is allowed to go below zero. */
    private void changeSelectedSetting(double amount) {
        if (selectedSetting == 0) targetVelocity = Math.max(0, targetVelocity + amount);
        if (selectedSetting == 1) kP = Math.max(0, kP + amount);
        if (selectedSetting == 2) kI = Math.max(0, kI + amount);
        if (selectedSetting == 3) kD = Math.max(0, kD + amount);
        if (selectedSetting == 4) kF = Math.max(0, kF + amount);
        applyPIDF();
    }

    /** Sends P, I, D and F to the hub. */
    private void applyPIDF() {
        launcher.setVelocityPIDFCoefficients(kP, kI, kD, kF);
    }

    /** Runs the flywheel and the windmill, and takes the measurements. */
    private void runFlywheel(double velocity) {
        if (flywheelOn) {
            launcher.setVelocity(targetVelocity);
        } else {
            launcher.setVelocity(0);
        }

        // ---- Spin-up: how long until we first get close to the target ----
        if (flywheelOn) {
            highestVelocity = Math.max(highestVelocity, velocity);
            if (!reachedTarget && Math.abs(velocity - targetVelocity) < TOLERANCE) {
                reachedTarget = true;
                spinUpSeconds = spinUpTimer.seconds();
            }
        }

        // ---- Wobble: the lowest and highest speed in each 1 second window ----
        wobbleLow = Math.min(wobbleLow, velocity);
        wobbleHigh = Math.max(wobbleHigh, velocity);
        if (wobbleTimer.seconds() > 1.0) {
            lastWobbleLow = wobbleLow;
            lastWobbleHigh = wobbleHigh;
            wobbleLow = Double.MAX_VALUE;
            wobbleHigh = 0;
            wobbleTimer.reset();
        }

        // ---- One shot: feed one ball, then watch the speed dip and come back ----
        if (shotInProgress) {
            double t = shotTimer.seconds();

            if (t < FEED_ONE_BALL_SECONDS) {
                windmillServo.setPower(1.0);
            } else {
                windmillServo.setPower(0);
            }

            lowestAfterShot = Math.min(lowestAfterShot, velocity);

            // Recovered = back within tolerance after the ball has left.
            if (!recovered && t > FEED_ONE_BALL_SECONDS
                    && Math.abs(velocity - targetVelocity) < TOLERANCE) {
                recovered = true;
                recoverySeconds = t;
            }

            // Give up waiting after 3 seconds.
            if (recovered || t > 3.0) {
                shotInProgress = false;
                windmillServo.setPower(0);
            }
        } else {
            windmillServo.setPower(0);
        }
    }

    /**
     * Find F: run at full power, see how fast the wheel can go, and work out F from that.
     *
     * The hub's velocity control uses 32767 to mean "full power". So if full power gives a top
     * speed of, say, 2600 ticks/s, then F = 32767 / 2600 = 12.6 is the F that would ask for
     * exactly the right power at every speed. P and I then start at 10% of the number before.
     * This is the method in REV's and FIRST's motor tuning guides.
     */
    private void runFindFTest(double velocity) {
        windmillServo.setPower(0);
        launcher.setPower(1.0);  // every loop, so nothing else can turn it down

        // Only trust the speed after the wheel has had 2 seconds to get up to full speed.
        if (findFTimer.seconds() > 2.0) {
            fastestSeen = Math.max(fastestSeen, velocity);
        }

        if (findFTimer.seconds() > 3.0) {
            findingF = false;
            launcher.setPower(0);
            launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);  // back to velocity control
            flywheelOn = false;

            if (fastestSeen > 0) {
                kF = 32767 / fastestSeen;
                kP = 0.1 * kF;
                kI = 0.1 * kP;
                kD = 0;
                applyPIDF();
            }
        }
    }

    private void showTelemetry(double velocity) {
        if (findingF) {
            telemetry.addLine("FIND F TEST RUNNING - full power, hands clear!");
            telemetry.addData("Time", "%.1f of 3.0 s", findFTimer.seconds());
            telemetry.addData("Velocity now", "%.0f ticks/s", velocity);
            return;
        }

        telemetry.addData("Flywheel", flywheelOn ? "ON" : "OFF");
        telemetry.addData("Velocity", "%.0f / %.0f ticks/s   (%.0f RPM)",
                velocity, targetVelocity, velocity / 28.0 * 60.0);
        telemetry.addData("Error", "%.0f ticks/s", targetVelocity - velocity);
        telemetry.addLine();

        // Show all five settings, with an arrow on the one the d-pad changes.
        double[] values = { targetVelocity, kP, kI, kD, kF };
        for (int i = 0; i < SETTING_NAMES.length; i++) {
            String arrow = (i == selectedSetting) ? ">> " : "   ";
            telemetry.addData(arrow + SETTING_NAMES[i], "%.4f", values[i]);
        }
        telemetry.addData("Step", "%.4f   (d-pad left/right to change)", step);
        telemetry.addLine();

        telemetry.addLine("--- Spin-up (press A) ---");
        telemetry.addData("Spin-up time", reachedTarget ? String.format("%.2f s", spinUpSeconds) : "not there yet");
        telemetry.addData("Overshoot", "%.0f ticks/s", Math.max(0, highestVelocity - targetVelocity));
        telemetry.addData("Wobble", "%.0f to %.0f  (+/- %.0f)",
                lastWobbleLow, lastWobbleHigh, (lastWobbleHigh - lastWobbleLow) / 2);
        telemetry.addLine();

        telemetry.addLine("--- One shot (press X) ---");
        if (lowestAfterShot > 0) {
            telemetry.addData("Lowest after shot", "%.0f ticks/s  (dip of %.0f)",
                    lowestAfterShot, targetVelocity - lowestAfterShot);
            telemetry.addData("Recovery time", recovered ? String.format("%.2f s", recoverySeconds) : "-");
        } else {
            telemetry.addLine("Spin up, load a ball, then press X.");
        }
    }

    @Override
    public void stop() {
        launcher.setPower(0);
        intake.setPower(0);
        windmillServo.setPower(0);
        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);
    }
}
