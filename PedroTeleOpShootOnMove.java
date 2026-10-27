package org.firstinspires.ftc.teamcode;

import static com.pedropathing.api.Paths.line;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.controllers.Controller;
import com.pedropathing.controllers.PIDController;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;

/*
 * PEDRO TELEOP - SHOOT ON THE MOVE. Everything "4b - Pedro TeleOp (Advanced)" does, but AIM
 * MODE now works while the robot is driving:
 *
 *   1. A LOOKUP TABLE picks the flywheel speed from the DISTANCE to the cell. Our launcher has a
 *      fixed angle, so speed is the only thing we can change to make the ball land at a different
 *      distance. The table was MEASURED by firing one ball at a time from many distances and
 *      speeds and writing down which ones scored. Between two rows we INTERPOLATE: 48 in is half
 *      way between 46 and 50, so it gets the speed half way between theirs.
 *
 *   2. LEADING THE TARGET. A ball leaves the robot with the ROBOT'S velocity added to its own,
 *      just like a ball thrown from a moving car. If the robot is moving right at 30 in/s and the
 *      ball takes 0.4 s to get there, the ball lands 12 in to the right. So instead of aiming at
 *      the real cell, we aim at a VIRTUAL target: the cell moved back the other way by
 *      (robot velocity x flight time). The flight time depends on the distance, which depends on
 *      where we aim... so we do the sum three times and it settles on the answer.
 *
 *   3. FIRE ONLY WHEN IT WILL GO IN. While aiming, holding the shoot button only feeds a ball when
 *      the robot is pointing at the virtual target, the flywheel is at the table's speed, and the
 *      distance is one the table knows. Otherwise it waits - and the screen says why.
 *
 * GAMEPAD 1 - DRIVER
 *   Left stick / right stick   drive / turn             Right bumper (hold)  slow mode
 *   Left bumper (hold)         AIM MODE: turn to the target, set the flywheel speed, keep driving
 *   D-pad up                   DRIVE TO SHOT spot       Y                    field / robot-centric
 *   X / B                      BLUE / RED alliance      A                    reset heading
 *   D-pad left                 "the robot is on the autonomous start spot" (sets the position)
 *
 * GAMEPAD 2 - OPERATOR
 *   Right trigger / left trigger   intake in / out
 *   Y                              flywheel on / off (outside aim mode)
 *   Right bumper (hold)            SHOOT. In aim mode it waits until the shot is lined up.
 *   Left bumper (hold)             windmill backward to clear a jam
 *   D-pad up / down                flywheel speed +/- 25 (outside aim mode)
 */
@TeleOp(name = "4c - Pedro TeleOp (Shoot on the Move)", group = "Competition")
public class PedroTeleOpShootOnMove extends OpMode {

    // ============================================================================================
    // THE LOOKUP TABLE
    //
    // One row per measured distance: { distance to the cell opening (in),
    //                                  flywheel speed that scores (ticks/s),
    //                                  how long the ball takes to get there (s) }
    //
    // Measured in the FTCKesselRun simulator: for each distance, every speed from 900 to 1600 in
    // steps of 25 was tried, and the middle of the range that scored is written here. The flight
    // times come from the simulator's ball physics.
    //
    // THE REAL ROBOT NEEDS ITS OWN TABLE. Stand at each distance, use the Flywheel Tuner to find
    // the lowest and highest speeds that score, and write down the middle. Flight time: film a
    // shot with a phone in slow motion and count frames from launch to the cell.
    //
    // Rows must be in order of distance. Notice the speed goes DOWN as the distance goes up: with
    // our steep launcher the ball is still near the top of its arc when it reaches the cell.
    // ============================================================================================
    private static final double[][] SHOT_TABLE = {
            { 42, 1410, 0.27 },
            { 46, 1290, 0.33 },
            { 50, 1240, 0.37 },
            { 54, 1200, 0.42 },
            { 58, 1175, 0.46 },
            { 64, 1175, 0.52 },
    };
    private static final int DISTANCE = 0, SPEED = 1, FLIGHT_TIME = 2;   // the columns

    /*
     * How much of the robot's velocity the ball keeps when it leaves.
     *   1.0  real physics: the ball keeps all of it. USE THIS ON THE REAL ROBOT.
     *   0.0  no leading at all.
     * THE SIMULATOR DOES NOT YET GIVE THE BALL THE ROBOT'S VELOCITY, so in FTCKesselRun set this
     * to 0 - otherwise the robot will aim off to the side and miss, because the simulated ball
     * flies as if the robot were standing still.
     */
    private static final double VELOCITY_LEAD = 1.0;

    // How close everything must be before aim mode will fire.
    private static final double AIM_TOLERANCE_DEGREES = 2.0;
    private static final double SPEED_TOLERANCE = 30;   // ticks per second

    // ---- Pedro drives the wheels ----
    private Follower follower;

    // ---- Mechanisms ----
    private DcMotorEx launcher;
    private DcMotor intake;
    private CRServo leftIntakeServo;
    private CRServo rightIntakeServo;
    private CRServo windmillServo;

    // ---- Launcher and shot detection ----
    private double launcherTargetVelocity = 1250;   // used outside aim mode
    private static final PIDFCoefficients LAUNCHER_PIDF = new PIDFCoefficients(40, 0, 0, 12.5);
    private static final double SHOT_DIP = 30;
    private static final double FEED_TIMEOUT = 0.75;
    private static final double UNJAM_SECONDS = 0.25;

    private enum ShooterState { WAITING_TO_FIRE, FEEDING, UNJAMMING, EMPTY }
    private ShooterState shooterState = ShooterState.WAITING_TO_FIRE;
    private final ElapsedTime shooterTimer = new ElapsedTime();
    private double velocityBeforeFeed = 0;
    private boolean triedUnjam = false;
    private int ballsShot = 0;
    private boolean wasReady = false;

    // ---- What aim mode worked out this loop (shown on the screen) ----
    private boolean aiming = false;
    private double aimHeading = 0;       // radians, toward the virtual target
    private double aimDistance = 0;      // inches, to the virtual target
    private double aimSpeed = 0;         // flywheel ticks/s from the table
    private double leadInches = 0;       // how far the virtual target is from the real cell
    private String notReadyBecause = "";

    // ---- Driving ----
    private static final double SLOW_MODE_SPEED = 0.4;
    private static final double STRAFE_MULTIPLIER = 1.25;   // T2 / T3 in "0 - Calibration Test"
    private boolean fieldCentric = true;
    private boolean blueAlliance = true;
    private boolean flywheelOn = false;
    private boolean positionKnown = false;
    // AIM: P and D for Pedro's PID controller, plus a feedforward in turn power per (radian per
    // second). 0.22 is 0.5 power / 2.24 rad/s, the turn rate "0 - Calibration Test" T4 measures.
    private final PIDController aimController = Controller.pid(2.0, 0, 0.1);
    private static final double AIM_FEEDFORWARD = 0.22;
    private double aimHeadingRate = 0;
    private double lastAimHeading = 0;
    private boolean wasAiming = false;
    private final ElapsedTime aimRateTimer = new ElapsedTime();

    // ---- Drive to shot ----
    private boolean autoDriving = false;
    private final ElapsedTime autoDriveTimer = new ElapsedTime();
    private static final double AUTO_DRIVE_TIMEOUT = 4.0;

    // ---- End of match ----
    private final ElapsedTime matchTimer = new ElapsedTime();
    private static final double TELEOP_SECONDS = 120;
    private boolean parkReminderDone = false;

    // ---- Places on the field, written for BLUE; RED is turned 180 degrees ----
    private PoseFactory makeFactory() {
        PoseFactory p = PoseFactory.degrees();
        if (!blueAlliance) {
            p = p.mirrorAroundPoint(72, 72);
        }
        return p;
    }

    private Pose startPose()      { return makeFactory().of(120, 135, -90); }
    private Pose farShotPose()    { return makeFactory().of(120, 129, -131); }
    private Pose nearShotPose()   { return makeFactory().of(120, 18, 133); }
    private Pose farCell()        { return makeFactory().of(84.5, 88, 0); }
    private Pose audienceCell()   { return makeFactory().of(85, 56, 0); }

    private boolean onFarSide() {
        return follower.pose().y() > 72;
    }

    // ============================================================================================
    // INIT
    // ============================================================================================

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        if (PedroAutoAdvanced.lastPose != null) {
            follower.setPose(PedroAutoAdvanced.lastPose);
            positionKnown = true;
        }

        launcher         = hardwareMap.get(DcMotorEx.class, "launcher");
        intake           = hardwareMap.get(DcMotor.class, "intake");
        leftIntakeServo  = hardwareMap.get(CRServo.class, "left_intake_servo");
        rightIntakeServo = hardwareMap.get(CRServo.class, "right_intake_servo");
        windmillServo    = hardwareMap.get(CRServo.class, "windmillServo");

        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        windmillServo.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, LAUNCHER_PIDF);
    }

    @Override
    public void init_loop() {
        if (gamepad1.xWasPressed()) blueAlliance = true;
        if (gamepad1.bWasPressed()) blueAlliance = false;
        telemetry.addData("Alliance", blueAlliance ? "BLUE" : "RED");
        if (positionKnown) {
            telemetry.addLine("Position: from the autonomous.");
        } else {
            telemetry.addLine("Position UNKNOWN. Put the robot on the auto start spot, press START,");
            telemetry.addLine("then press d-pad left.");
        }
        if (VELOCITY_LEAD == 0) telemetry.addLine("VELOCITY_LEAD is 0 (simulator setting).");
    }

    @Override
    public void start() {
        matchTimer.reset();
    }

    // ============================================================================================
    // LOOP
    // ============================================================================================

    @Override
    public void loop() {
        aiming = gamepad1.left_bumper && positionKnown;
        if (aiming) {
            workOutTheShot();
        }
        wasAiming = aiming;

        drive();
        runIntakeAndShooter();
        endOfMatchReminder();

        follower.update();

        Pose pose = follower.pose();
        telemetry.addData("Drive", "%s%s", fieldCentric ? "FIELD-centric" : "ROBOT-centric",
                autoDriving ? "  >> DRIVING TO SHOT <<" : "");
        telemetry.addData("Position", "x %.1f  y %.1f  heading %.0f%s", pose.x(), pose.y(),
                Math.toDegrees(pose.heading()), positionKnown ? "" : "  (UNKNOWN - d-pad left)");
        if (aiming) {
            telemetry.addData("AIM", "%.0f in, lead %.1f in, speed %.0f", aimDistance, leadInches, aimSpeed);
            telemetry.addData("Ready", readyToFire() ? "YES" : notReadyBecause);
        }
        telemetry.addData("Flywheel", "%.0f ticks/s", launcher.getVelocity());
        telemetry.addData("Shooter", "%s   shots: %d", shooterState, ballsShot);
        telemetry.addData("Time left", "%.0f s", TELEOP_SECONDS - matchTimer.seconds());
    }

    // ============================================================================================
    // AIMING: WHERE TO POINT AND HOW FAST TO SPIN
    // ============================================================================================

    /** Fills in aimHeading, aimDistance, aimSpeed and leadInches for this loop. */
    private void workOutTheShot() {
        Pose robot = follower.pose();
        Velocity velocity = follower.velocity();   // field directions, inches per second
        Pose cell = onFarSide() ? farCell() : audienceCell();

        // Start by aiming at the real cell, then improve the guess three times.
        double targetX = cell.x();
        double targetY = cell.y();
        for (int i = 0; i < 3; i++) {
            double distance = Math.hypot(targetX - robot.x(), targetY - robot.y());
            double flightTime = lookUp(distance, FLIGHT_TIME);

            // While the ball flies, the robot's velocity carries it this far. Aim that far the
            // OTHER way, and the robot's motion brings the ball back onto the cell.
            targetX = cell.x() - velocity.vx * flightTime * VELOCITY_LEAD;
            targetY = cell.y() - velocity.vy * flightTime * VELOCITY_LEAD;
        }

        aimHeading  = Math.atan2(targetY - robot.y(), targetX - robot.x());
        aimDistance = Math.hypot(targetX - robot.x(), targetY - robot.y());

        // How fast the aim direction is turning (radians per second), for the feedforward.
        // Smoothed a little, because it is worked out from two noisy measurements in a row.
        double change = aimHeading - lastAimHeading;
        while (change > Math.PI) change -= 2 * Math.PI;
        while (change < -Math.PI) change += 2 * Math.PI;
        double dt = aimRateTimer.seconds();
        aimRateTimer.reset();
        if (wasAiming && dt > 0) {
            aimHeadingRate = 0.7 * aimHeadingRate + 0.3 * (change / dt);
        } else {
            aimHeadingRate = 0;   // just started aiming: no rate yet
        }
        lastAimHeading = aimHeading;
        aimSpeed    = lookUp(aimDistance, SPEED);
        leadInches  = Math.hypot(targetX - cell.x(), targetY - cell.y());
    }

    /**
     * Reads one column of the table at any distance, by INTERPOLATING between the two rows on
     * either side of it. Outside the table it uses the nearest row (and aim mode will not fire).
     *
     * Example: distance 48 is between the rows for 46 and 50. It is 2 of the 4 inches along, so
     * the answer is 2/4 = half way between those rows' numbers.
     */
    private double lookUp(double distance, int column) {
        int last = SHOT_TABLE.length - 1;
        if (distance <= SHOT_TABLE[0][DISTANCE])    return SHOT_TABLE[0][column];
        if (distance >= SHOT_TABLE[last][DISTANCE]) return SHOT_TABLE[last][column];

        for (int i = 0; i < last; i++) {
            double[] below = SHOT_TABLE[i];
            double[] above = SHOT_TABLE[i + 1];
            if (distance <= above[DISTANCE]) {
                double fraction = (distance - below[DISTANCE]) / (above[DISTANCE] - below[DISTANCE]);
                return below[column] + fraction * (above[column] - below[column]);
            }
        }
        return SHOT_TABLE[last][column];   // never reached, but Java wants a return here
    }

    /** True when an aimed shot will go in. If not, notReadyBecause says why. */
    private boolean readyToFire() {
        double headingError = Math.toDegrees(aimHeading - follower.pose().heading());
        while (headingError > 180) headingError -= 360;
        while (headingError < -180) headingError += 360;

        if (aimDistance < SHOT_TABLE[0][DISTANCE]) {
            notReadyBecause = "TOO CLOSE - back up";
        } else if (aimDistance > SHOT_TABLE[SHOT_TABLE.length - 1][DISTANCE]) {
            notReadyBecause = "TOO FAR - get closer";
        } else if (Math.abs(headingError) > AIM_TOLERANCE_DEGREES) {
            notReadyBecause = String.format("turning (%.1f deg off)", headingError);
        } else if (Math.abs(launcher.getVelocity() - aimSpeed) > SPEED_TOLERANCE) {
            notReadyBecause = String.format("flywheel %.0f of %.0f", launcher.getVelocity(), aimSpeed);
        } else {
            notReadyBecause = "";
        }
        return notReadyBecause.isEmpty();
    }

    // ============================================================================================
    // DRIVING (gamepad 1)
    // ============================================================================================

    private void drive() {
        if (gamepad1.yWasPressed()) fieldCentric = !fieldCentric;
        if (gamepad1.xWasPressed()) blueAlliance = true;
        if (gamepad1.bWasPressed()) blueAlliance = false;
        if (gamepad1.aWasPressed()) follower.setHeading(driverForwardHeading());
        if (gamepad1.dpadLeftWasPressed()) {
            follower.setPose(startPose());
            positionKnown = true;
        }

        double forward = -gamepad1.left_stick_y;
        double left    = -gamepad1.left_stick_x;
        double turn    = -gamepad1.right_stick_x;
        boolean sticksMoved = Math.abs(forward) > 0.2 || Math.abs(left) > 0.2 || Math.abs(turn) > 0.2;

        // ---- Drive to shot: Pedro drives until it arrives, a stick moves, or time runs out ----
        if (gamepad1.dpadUpWasPressed() && positionKnown) {
            Pose here = follower.pose();
            Pose target = onFarSide() ? farShotPose() : nearShotPose();
            follower.follow(line(here, target).linear(here, target));
            autoDriving = true;
            autoDriveTimer.reset();
        }
        if (autoDriving) {
            if (sticksMoved || !follower.isBusy() || autoDriveTimer.seconds() > AUTO_DRIVE_TIMEOUT) {
                autoDriving = false;
            } else {
                return;
            }
        }

        if (gamepad1.right_bumper) {
            forward *= SLOW_MODE_SPEED;
            left    *= SLOW_MODE_SPEED;
            turn    *= SLOW_MODE_SPEED;
        }

        DrivePowers powers;
        if (fieldCentric) {
            double angle = follower.pose().heading() - driverForwardHeading();
            powers = ManualDrive.fieldCentric(forward, left, turn, angle);
        } else {
            powers = new DrivePowers(forward, left, turn);
        }
        powers = new DrivePowers(powers.forward(), powers.strafe() * STRAFE_MULTIPLIER, powers.turn());

        // ---- AIM MODE: the code does the turning, toward the VIRTUAL target ----
        if (aiming) {
            powers = new DrivePowers(powers.forward(), powers.strafe(), aimTurnPower());
        } else {
            aimController.reset();
        }

        follower.manual(powers);
    }

    /**
     * How hard to turn to point at the target. Three parts:
     *   P            turn toward the target, harder the further off we are
     *   D            ease off as we close in, so we do not overshoot
     *   FEEDFORWARD  while we drive, the direction to the target keeps changing. Turn at that
     *                same rate, so P only has to fix what is left. Without this the robot always
     *                trails a few degrees behind a moving target - too far behind to fire.
     *
     * (Pedro's ManualDrive.headingLock does the P and D parts. We do it ourselves here because
     * it has no feedforward, and its D part fights the robot turning at all.)
     */
    private double aimTurnPower() {
        double error = aimHeading - follower.pose().heading();
        while (error > Math.PI) error -= 2 * Math.PI;
        while (error < -Math.PI) error += 2 * Math.PI;

        // Pedro's PID controller: P on the error, D on how fast the error is changing.
        double feedback = aimController.calculate(aimHeading, error);
        return feedback + AIM_FEEDFORWARD * aimHeadingRate;
    }

    private double driverForwardHeading() {
        return blueAlliance ? Math.PI : 0;
    }

    // ============================================================================================
    // INTAKE AND SHOOTER (gamepad 2)
    // ============================================================================================

    private void runIntakeAndShooter() {
        if (gamepad2.yWasPressed())        flywheelOn = !flywheelOn;
        if (gamepad2.dpadUpWasPressed())   launcherTargetVelocity += 25;
        if (gamepad2.dpadDownWasPressed()) launcherTargetVelocity -= 25;

        boolean shooting = gamepad2.right_bumper;
        if (gamepad2.rightBumperWasPressed()) {
            shooterState = ShooterState.WAITING_TO_FIRE;
            triedUnjam = false;
            shooterTimer.reset();
        }

        // ---- Flywheel: in aim mode the TABLE sets the speed ----
        double targetSpeed = aiming ? aimSpeed : launcherTargetVelocity;
        if (aiming || flywheelOn || shooting) {
            launcher.setVelocity(targetSpeed);
        } else {
            launcher.setVelocity(0);
        }
        double velocity = launcher.getVelocity();

        boolean ready = (aiming || flywheelOn || shooting) && Math.abs(velocity - targetSpeed) < SPEED_TOLERANCE;
        if (ready && !wasReady) gamepad2.rumble(200);
        wasReady = ready;

        // ---- Windmill ----
        if (shooting) {
            // In aim mode, fire only when the shot is lined up. Otherwise, when up to speed.
            boolean clearToFire = aiming ? readyToFire() : velocity >= targetSpeed;
            runShooter(velocity, clearToFire);
        } else if (gamepad2.left_bumper) {
            windmillServo.setPower(-1.0);
        } else {
            windmillServo.setPower(0);
        }

        double intakePower = gamepad2.right_trigger - gamepad2.left_trigger;
        intake.setPower(intakePower);
        leftIntakeServo.setPower(intakePower);
        rightIntakeServo.setPower(intakePower);
    }

    /** One ball at a time, watching for the flywheel dip that says each ball left. */
    private void runShooter(double velocity, boolean clearToFire) {
        switch (shooterState) {
            case WAITING_TO_FIRE:
                windmillServo.setPower(0);
                if (clearToFire) {
                    velocityBeforeFeed = velocity;
                    shooterState = ShooterState.FEEDING;
                    shooterTimer.reset();
                }
                break;

            case FEEDING:
                windmillServo.setPower(1.0);
                if (velocity < velocityBeforeFeed - SHOT_DIP) {
                    windmillServo.setPower(0);
                    ballsShot++;
                    triedUnjam = false;
                    shooterState = ShooterState.WAITING_TO_FIRE;
                } else if (shooterTimer.seconds() > FEED_TIMEOUT) {
                    windmillServo.setPower(0);
                    if (!triedUnjam) {
                        triedUnjam = true;
                        shooterState = ShooterState.UNJAMMING;
                        shooterTimer.reset();
                    } else {
                        shooterState = ShooterState.EMPTY;
                        gamepad2.rumbleBlips(2);
                    }
                }
                break;

            case UNJAMMING:
                windmillServo.setPower(-1.0);
                if (shooterTimer.seconds() > UNJAM_SECONDS) {
                    windmillServo.setPower(0);
                    shooterState = ShooterState.WAITING_TO_FIRE;
                }
                break;

            case EMPTY:
                windmillServo.setPower(0);
                break;
        }
    }

    // ============================================================================================
    // END OF MATCH
    // ============================================================================================

    private void endOfMatchReminder() {
        if (!parkReminderDone && matchTimer.seconds() > TELEOP_SECONDS - 10) {
            gamepad1.rumbleBlips(3);
            parkReminderDone = true;
        }
    }

    @Override
    public void stop() {
        follower.stop();
        launcher.setPower(0);
        intake.setPower(0);
        windmillServo.setPower(0);
        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);
    }
}
