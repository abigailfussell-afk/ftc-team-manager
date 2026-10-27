package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

/*
 * CALIBRATION TEST - is the REAL robot doing what the SIMULATED robot does?
 *
 * Every test here runs the robot in a fixed, repeatable way (the same power for the same time)
 * and measures what happened with the Pinpoint and the motor encoders. The same tests were run
 * on the simulated robot in FTCKesselRun, and those numbers are written into this file as the
 * SIM_ values below. Each result is shown next to the simulator's number.
 *
 *   OK    the real robot is within 20% of the simulator
 *   DIFF  the real robot is more than 20% away. The simulator needs adjusting (or the robot does)
 *   FAIL  something is wrong that no simulator setting explains: a backward motor, a pod that
 *         reads the wrong way, a dead encoder. Fix the robot first.
 *
 * BEFORE YOU START
 *   - Fully charged battery. Results depend on battery voltage, so it goes in the report.
 *   - Robot on the FIELD TILES (not on a block), in open space: about 5 ft clear in front,
 *     4 ft clear to the right, and room to spin. Mark where it starts with tape.
 *   - Do not touch the robot while a test runs.
 *
 * GAMEPAD 1
 *   D-pad up / down   pick a test (or ALL)
 *   A                 run it
 *   B                 STOP the test right now
 *   X                 clear the report
 *
 * GETTING THE RESULTS BACK
 *   The report is on the Driver Hub screen after each test. Take a photo of it, or copy the
 *   file /sdcard/FIRST/calibration-report.txt off the Control Hub. Each line starts with a
 *   test number, so the report can be pasted or typed back exactly as it is.
 */
@TeleOp(name = "0 - Calibration Test", group = "Test")
public class CalibrationTest extends LinearOpMode {

    // ============================================================================================
    // WHAT THE SIMULATOR MEASURED
    // FTCKesselRun, robots/biobuzz-starterwithodo.robot.json, BIOBUZZ field, starting at
    // heading 0 on the tiles, battery 12.4 V. Change these if you change the simulated robot.
    // ============================================================================================
    static final double SIM_WHEEL_VELOCITY = 823;   // T1: ticks/s, one wheel at 30% power
    static final double SIM_FORWARD_DIST   = 32.3;  // T2: inches, 50% forward for 1 s
    static final double SIM_FORWARD_PEAK   = 32.7;  // T2: inches/s
    static final double SIM_FORWARD_COAST  = 6.1;   // T2: inches travelled after the power stops
    static final double SIM_STRAFE_DIST    = 25.9;  // T3: inches, 50% strafe right for 1 s
    static final double SIM_TURN_DEGREES   = 128.1; // T4: degrees, 50% turn clockwise for 1 s
    static final double SIM_TURN_PEAK      = 128.6; // T4: degrees/s
    static final double SIM_SPIN_DRIFT     = 0.0;   // T5: inches the Pinpoint wandered in 3 spins
    static final double SIM_SPINUP_SECONDS = 0.90;  // T6: seconds to reach 1250 ticks/s
    static final double SIM_STEADY_VEL     = 1278;  // T6: ticks/s once settled at a 1250 target
    static final double SIM_TOP_SPEED      = 2749;  // T6: ticks/s at full power

    static final double TOLERANCE = 0.20;           // OK if within 20% of the simulator

    // ---- Hardware ----
    private DcMotorEx leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive;
    private DcMotorEx launcher;
    private DcMotor intake;
    private CRServo windmillServo, leftIntakeServo, rightIntakeServo;
    private GoBildaPinpointDriver pinpoint;

    // ---- The menu ----
    private final String[] TESTS = {
            "ALL (T1 to T6, about 30 s)",
            "T1 Wheels: each wheel alone",
            "T2 Drive forward",
            "T3 Strafe right",
            "T4 Turn clockwise",
            "T5 Spin 3 times (odometry)",
            "T6 Flywheel"
    };
    private final String[] INSTRUCTIONS = {
            "Needs 5 ft in front, 4 ft to the right, room to spin.",
            "Each wheel spins on its own for 1 s. The robot will wiggle.",
            "Drives about 3 ft FORWARD. Needs 5 ft clear in front.",
            "Strafes about 2 ft RIGHT. Needs 4 ft clear to the right.",
            "Turns about 1/3 of a turn clockwise on the spot.",
            "Spins 3 full turns on the spot. Mark the start with tape!",
            "Spins the launcher. Keep hands clear. Robot does not move."
    };
    private int selectedTest = 0;

    // ---- The report: one line per finding ----
    private final List<String> report = new ArrayList<>();

    private final ElapsedTime timer = new ElapsedTime();
    private boolean stoppedByDriver = false;
    private double forwardDistance = 0;   // from T2, for the strafe multiplier in T3

    @Override
    public void runOpMode() {
        initHardware();

        telemetry.addLine("Calibration Test ready.");
        telemetry.addLine("Robot on the tiles, in open space. Press START.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // ---- The menu ----
            if (gamepad1.dpadDownWasPressed()) selectedTest = (selectedTest + 1) % TESTS.length;
            if (gamepad1.dpadUpWasPressed())   selectedTest = (selectedTest + TESTS.length - 1) % TESTS.length;
            if (gamepad1.xWasPressed())        report.clear();

            if (gamepad1.aWasPressed()) {
                runSelectedTest();
                saveReport();
            }

            showMenuAndReport();
        }
        stopEverything();
    }

    private void runSelectedTest() {
        stoppedByDriver = false;
        addLine(String.format("CAL battery %.2f V", batteryVoltage()));
        if (selectedTest == 0 || selectedTest == 1) { showRunning(1); testWheels(); }
        if (selectedTest == 0 || selectedTest == 2) { showRunning(2); testForward(); }
        if (selectedTest == 0 || selectedTest == 3) { showRunning(3); testStrafe(); }
        if (selectedTest == 0 || selectedTest == 4) { showRunning(4); testTurn(); }
        if (selectedTest == 0 || selectedTest == 5) { showRunning(5); testSpin(); }
        if (selectedTest == 0 || selectedTest == 6) { showRunning(6); testFlywheel(); }
        stopEverything();
    }

    private void showRunning(int test) {
        telemetry.addLine("RUNNING " + TESTS[test]);
        telemetry.addLine("Press B to STOP.");
        telemetry.update();
    }

    // ============================================================================================
    // THE TESTS
    // ============================================================================================

    /**
     * T1: each wheel on its own at 30% for 1 second.
     * Checks every wheel spins FORWARD (positive velocity) and that all four are about equal.
     */
    private void testWheels() {
        String[] names = { "lf", "rf", "lb", "rb" };
        DcMotorEx[] wheels = { leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive };
        double[] velocities = new double[4];

        for (int i = 0; i < 4 && keepGoing(); i++) {
            wheels[i].setPower(0.3);
            pause(0.7);                              // let it get up to speed
            velocities[i] = averageVelocity(wheels[i], 0.3);
            wheels[i].setPower(0);
            pause(0.4);                              // let it stop before the next wheel
        }
        if (!keepGoing()) return;

        double slowest = Double.MAX_VALUE, fastest = -Double.MAX_VALUE;
        for (double v : velocities) {
            slowest = Math.min(slowest, v);
            fastest = Math.max(fastest, v);
        }

        addLine(String.format("T1 lf %.0f rf %.0f lb %.0f rb %.0f t/s",
                velocities[0], velocities[1], velocities[2], velocities[3]));
        for (int i = 0; i < 4; i++) {
            if (velocities[i] <= 10) {
                addLine("T1 FAIL " + names[i] + " not forward: backward motor, or encoder unplugged");
            }
        }
        addLine("T1 " + compare("wheel speed", (slowest + fastest) / 2, SIM_WHEEL_VELOCITY));
        if (slowest > 10 && fastest > slowest * 1.15) {
            addLine("T1 DIFF wheels differ by more than 15%: check for rubbing or a tight wheel");
        }
    }

    /** T2: 50% forward for 1 second, then let go. */
    private void testForward() {
        Move move = timedMove(0.5, 0, 0, 1.0);
        if (move == null) return;

        forwardDistance = move.forward;   // T3 uses this to work out the strafe multiplier
        addLine(String.format("T2 fwd %.1f in, side %.1f in, turned %.1f deg",
                move.forward, move.left, move.degrees));
        addLine("T2 " + compare("distance", move.forward, SIM_FORWARD_DIST));
        addLine("T2 " + compare("top speed", move.peakSpeed, SIM_FORWARD_PEAK));
        addLine("T2 " + compare("coast", move.coast, SIM_FORWARD_COAST));
        if (move.forward < 0) addLine("T2 FAIL went BACKWARD, or the forward pod reads backward");
        if (Math.abs(move.left) > 0.25 * Math.abs(move.forward)) {
            addLine("T2 DIFF drifted sideways: strafe pod direction, or a wheel slipping");
        }
    }

    /** T3: 50% strafe right for 1 second, then let go. */
    private void testStrafe() {
        Move move = timedMove(0, 0.5, 0, 1.0);
        if (move == null) return;

        double right = -move.left;
        addLine(String.format("T3 right %.1f in, fwd %.1f in, turned %.1f deg",
                right, move.forward, move.degrees));
        addLine("T3 " + compare("distance", right, SIM_STRAFE_DIST));
        if (right < 0) addLine("T3 FAIL went LEFT, or the strafe pod reads backward");

        // Strafing is weaker than driving forward. The TeleOps make up for it with
        // STRAFE_MULTIPLIER, which is just how many times further the robot went forward.
        if (forwardDistance > 0 && right > 0) {
            addLine(String.format("T3 set STRAFE_MULTIPLIER = %.2f in the TeleOps (sim %.2f)",
                    forwardDistance / right, SIM_FORWARD_DIST / SIM_STRAFE_DIST));
        }
        if (Math.abs(move.degrees) > 10) {
            addLine("T3 DIFF turned while strafing: check wheel order/rollers (X pattern from above)");
        }
    }

    /** T4: 50% turn clockwise for 1 second, then let go. */
    private void testTurn() {
        Move move = timedMove(0, 0, 0.5, 1.0);
        if (move == null) return;

        double clockwise = -move.degrees;   // headings grow counter-clockwise
        addLine(String.format("T4 turned %.1f deg CW, moved %.1f in",
                clockwise, Math.hypot(move.forward, move.left)));
        addLine("T4 " + compare("angle", clockwise, SIM_TURN_DEGREES));
        addLine("T4 " + compare("turn speed", move.peakTurnSpeed, SIM_TURN_PEAK));
        if (clockwise < 0) addLine("T4 FAIL turned the wrong way: drive motor directions");
    }

    /**
     * T5: spin counter-clockwise 3 full turns, then stop.
     *
     * The robot turns on the spot, so the Pinpoint should say it ends where it started. If it says
     * it moved a lot, the pod OFFSETS are wrong: a pod in a different place than the code says
     * makes every turn look like a little circle. Check the tape mark too: if the robot really
     * did end on its mark but the Pinpoint says it moved, it is the offsets.
     */
    private void testSpin() {
        pinpoint.update();
        double startX = pinpoint.getPosX(DistanceUnit.INCH);
        double startY = pinpoint.getPosY(DistanceUnit.INCH);
        double lastHeading = pinpoint.getHeading(AngleUnit.DEGREES);
        double totalTurned = 0;

        timer.reset();
        while (keepGoing() && totalTurned < 3 * 360 && timer.seconds() < 15) {
            setDrive(0, 0, -0.4);   // negative turn = counter-clockwise
            pinpoint.update();

            // Add up how far we turned since last loop. The heading jumps from +180 to -180
            // every half turn, so wrap the small step back into -180..180 before adding it.
            double heading = pinpoint.getHeading(AngleUnit.DEGREES);
            double step = heading - lastHeading;
            if (step > 180) step -= 360;
            if (step < -180) step += 360;
            totalTurned += step;
            lastHeading = heading;

            telemetry.addData("T5 spinning", "%.0f of 1080 deg", totalTurned);
            telemetry.update();
        }
        setDrive(0, 0, 0);
        pause(1.0);
        if (!keepGoing()) return;

        pinpoint.update();
        double drift = Math.hypot(pinpoint.getPosX(DistanceUnit.INCH) - startX,
                                  pinpoint.getPosY(DistanceUnit.INCH) - startY);
        addLine(String.format("T5 spun %.0f deg in %.1f s, Pinpoint drift %.1f in",
                totalTurned, timer.seconds(), drift));
        if (totalTurned < 3 * 360) addLine("T5 FAIL never finished 3 turns: heading not changing?");
        if (drift > 3) {
            addLine("T5 DIFF Pinpoint moved > 3 in during a spin: check the pod OFFSETS");
        } else {
            addLine("T5 OK drift " + String.format("%.1f in (sim %.1f)", drift, SIM_SPIN_DRIFT));
        }
    }

    /** T6: spin up to 1250 ticks/s with goBILDA's PIDF, hold it, then full power. */
    private void testFlywheel() {
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setVelocityPIDFCoefficients(40, 0, 0, 12.5);

        // 1. How long to reach the target?
        launcher.setVelocity(1250);
        timer.reset();
        double spinUp = -1;
        while (keepGoing() && timer.seconds() < 4) {
            if (spinUp < 0 && Math.abs(launcher.getVelocity() - 1250) < 25) spinUp = timer.seconds();
            if (spinUp >= 0) break;
            idle();
        }

        // 2. Where does it settle?
        pause(1.0);
        double steady = averageVelocity(launcher, 1.5);

        // 3. How fast can it go?
        launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcher.setPower(1.0);
        pause(2.5);
        double top = averageVelocity(launcher, 0.5);
        launcher.setPower(0);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (!keepGoing()) return;

        if (spinUp < 0) {
            addLine("T6 FAIL never reached 1250 t/s in 4 s");
        } else {
            addLine("T6 " + compare("spin-up s", spinUp, SIM_SPINUP_SECONDS));
        }
        addLine("T6 " + compare("steady t/s", steady, SIM_STEADY_VEL));
        addLine("T6 " + compare("top t/s", top, SIM_TOP_SPEED));
        if (top < 100) addLine("T6 FAIL launcher not turning, or its encoder is unplugged");
    }

    // ============================================================================================
    // HELPERS
    // ============================================================================================

    /** What one timed move did, measured by the Pinpoint, in the robot's own directions. */
    private static class Move {
        double forward, left, degrees;    // where it ended up, relative to where it started
        double peakSpeed;                 // inches per second
        double peakTurnSpeed;             // degrees per second
        double coast;                     // inches travelled after the power was cut
    }

    /**
     * Drives with fixed powers for a fixed time, lets go, waits for the robot to stop, and
     * reports what the Pinpoint saw. Returns null if B was pressed.
     */
    private Move timedMove(double forward, double strafe, double turn, double seconds) {
        pinpoint.update();
        double startX = pinpoint.getPosX(DistanceUnit.INCH);
        double startY = pinpoint.getPosY(DistanceUnit.INCH);
        double startHeading = pinpoint.getHeading(AngleUnit.RADIANS);
        Move move = new Move();

        // Drive.
        timer.reset();
        while (keepGoing() && timer.seconds() < seconds) {
            setDrive(forward, strafe, turn);
            pinpoint.update();
            double speed = Math.hypot(pinpoint.getVelX(DistanceUnit.INCH), pinpoint.getVelY(DistanceUnit.INCH));
            double turnSpeed = Math.abs(pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));
            move.peakSpeed = Math.max(move.peakSpeed, speed);
            move.peakTurnSpeed = Math.max(move.peakTurnSpeed, turnSpeed);
        }
        setDrive(0, 0, 0);
        pinpoint.update();
        double cutX = pinpoint.getPosX(DistanceUnit.INCH);
        double cutY = pinpoint.getPosY(DistanceUnit.INCH);

        // Let it stop.
        pause(1.0);
        if (!keepGoing()) return null;
        pinpoint.update();
        double endX = pinpoint.getPosX(DistanceUnit.INCH);
        double endY = pinpoint.getPosY(DistanceUnit.INCH);

        // Turn the field-frame movement into the robot's own forward / left at the start.
        double dx = endX - startX;
        double dy = endY - startY;
        move.forward =  dx * Math.cos(startHeading) + dy * Math.sin(startHeading);
        move.left    = -dx * Math.sin(startHeading) + dy * Math.cos(startHeading);
        move.degrees = Math.toDegrees(AngleUnit.normalizeRadians(
                pinpoint.getHeading(AngleUnit.RADIANS) - startHeading));
        move.coast = Math.hypot(endX - cutX, endY - cutY);
        return move;
    }

    /** "distance 22.1 (sim 22.4, 99%) OK" */
    private String compare(String what, double real, double sim) {
        if (sim == 0) {
            return String.format("%s %.2f (no sim value yet)", what, real);
        }
        double percent = real / sim * 100;
        String verdict = Math.abs(real - sim) <= TOLERANCE * Math.abs(sim) ? "OK" : "DIFF";
        return String.format("%s %s %.2f (sim %.2f, %.0f%%)", verdict, what, real, sim, percent);
    }

    /** Average velocity of a motor over the next few seconds. */
    private double averageVelocity(DcMotorEx motor, double seconds) {
        double total = 0;
        int samples = 0;
        ElapsedTime t = new ElapsedTime();
        while (keepGoing() && t.seconds() < seconds) {
            total += motor.getVelocity();
            samples++;
            idle();
        }
        return samples == 0 ? 0 : total / samples;
    }

    /** The same mecanum math as the TeleOps. + strafe is right, + turn is clockwise. */
    private void setDrive(double forward, double strafe, double turn) {
        leftFrontDrive.setPower(forward + strafe + turn);
        rightFrontDrive.setPower(forward - strafe - turn);
        leftBackDrive.setPower(forward - strafe + turn);
        rightBackDrive.setPower(forward + strafe - turn);
    }

    /** Waits, but keeps reading the Pinpoint and stops early if B is pressed. */
    private void pause(double seconds) {
        ElapsedTime t = new ElapsedTime();
        while (keepGoing() && t.seconds() < seconds) {
            pinpoint.update();
            idle();
        }
    }

    /**
     * False once the OpMode is stopped or the driver has pressed B. Once B is pressed it STAYS
     * false until the next run, so "ALL" stops completely instead of going on to the next test.
     */
    private boolean keepGoing() {
        if (gamepad1.b && !stoppedByDriver) {
            stoppedByDriver = true;
            stopEverything();
            addLine("STOPPED by driver (B) - results after this line are missing");
        }
        return opModeIsActive() && !stoppedByDriver;
    }

    private double batteryVoltage() {
        double lowest = Double.MAX_VALUE;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double v = sensor.getVoltage();
            if (v > 0) lowest = Math.min(lowest, v);
        }
        return lowest == Double.MAX_VALUE ? 0 : lowest;
    }

    private void addLine(String line) {
        report.add(line);
        RobotLog.ii("Calibration", line);   // also goes in the robot's log file
    }

    private void saveReport() {
        File file = new File(AppUtil.FIRST_FOLDER, "calibration-report.txt");
        file.getParentFile().mkdirs();   // make the FIRST folder if it is not there yet
        try (FileWriter writer = new FileWriter(file)) {
            writer.write(String.join("\n", report));
        } catch (IOException e) {
            report.add("(could not save the report file: " + e.getMessage() + ")");
        }
    }

    private void showMenuAndReport() {
        for (int i = 0; i < TESTS.length; i++) {
            telemetry.addLine((i == selectedTest ? ">> " : "   ") + TESTS[i]);
        }
        telemetry.addLine(INSTRUCTIONS[selectedTest]);
        telemetry.addLine("A run   B stop   X clear report");
        telemetry.addLine();
        telemetry.addLine("---- REPORT (also in /sdcard/FIRST/calibration-report.txt) ----");
        for (String line : report) telemetry.addLine(line);
        telemetry.update();
    }

    private void stopEverything() {
        setDrive(0, 0, 0);
        launcher.setPower(0);
        intake.setPower(0);
        windmillServo.setPower(0);
        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);
    }

    private void initHardware() {
        leftFrontDrive   = hardwareMap.get(DcMotorEx.class, "left_front_drive");
        rightFrontDrive  = hardwareMap.get(DcMotorEx.class, "right_front_drive");
        leftBackDrive    = hardwareMap.get(DcMotorEx.class, "left_back_drive");
        rightBackDrive   = hardwareMap.get(DcMotorEx.class, "right_back_drive");
        launcher         = hardwareMap.get(DcMotorEx.class, "launcher");
        intake           = hardwareMap.get(DcMotor.class, "intake");
        windmillServo    = hardwareMap.get(CRServo.class, "windmillServo");
        leftIntakeServo  = hardwareMap.get(CRServo.class, "left_intake_servo");
        rightIntakeServo = hardwareMap.get(CRServo.class, "right_intake_servo");
        pinpoint         = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // The SAME directions as every other OpMode, because that is what we are testing.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        windmillServo.setDirection(DcMotorSimple.Direction.REVERSE);
        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotorEx[] wheels = { leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive };
        for (DcMotorEx wheel : wheels) {
            wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            wheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // The same Pinpoint settings as pedro/Constants.java. Then zero it: from here on,
        // x points the way the robot faces right now.
        pinpoint.setOffsets(-0.25, -6.0, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                                      GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();
        sleep(500);   // the Pinpoint needs the robot still for a moment while it resets
    }
}
