package org.firstinspires.ftc.teamcode;

import static com.pedropathing.api.Paths.line;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.controllers.Controller;
import com.pedropathing.controllers.PIDController;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
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
 * PEDRO TELEOP - ADVANCED. Everything "4 - Pedro TeleOp" does, plus:
 *
 *   1. SHOT DETECTION. Holding the shoot button fires ONE ball at a time: the windmill runs until
 *      the flywheel DIPS (that means a ball just went through it), stops, waits for the wheel to
 *      be back at full speed, and goes again. It counts the shots, and if no ball comes out it
 *      backs the windmill up once to clear a jam, then says EMPTY.
 *   2. RUMBLE. Gamepad 2 buzzes once when the flywheel is up to speed and twice when the robot
 *      is EMPTY. Gamepad 1 buzzes when there are 10 seconds left, as a reminder to park.
 *   3. AIM LOCK. Hold gamepad 1's left bumper and the robot turns itself to face our HIVE's cell,
 *      while the driver keeps control of driving around.
 *   4. DRIVE TO SHOT. Press d-pad up and Pedro drives the robot to the shooting spot on whichever
 *      side of the HIVE it is on, then hands control back. Move a stick to cancel it at any time.
 *      It gives up after 4 seconds, so it can never get stuck pushing against something.
 *   5. WHERE AM I. Aim lock and drive-to-shot need to know where the robot is on the field.
 *      If "5b - Pedro Auto (Advanced)" ran first, we start from where it ended. If not, put the
 *      robot on the autonomous start spot and press d-pad left.
 *
 * GAMEPAD 1 - DRIVER
 *   Left stick / right stick   drive / turn             Right bumper (hold)  slow mode
 *   Left bumper (hold)         AIM LOCK at our cell     D-pad up             DRIVE TO SHOT spot
 *   Y                          field / robot-centric    X / B                BLUE / RED alliance
 *   A                          reset heading (point the robot straight away from you first)
 *   D-pad left                 "the robot is on the autonomous start spot" (sets the position)
 *
 * GAMEPAD 2 - OPERATOR
 *   Right trigger / left trigger   intake in / out
 *   Y                              flywheel on / off
 *   Right bumper (hold)            SHOOT, one ball at a time, until empty
 *   Left bumper (hold)             windmill backward to clear a jam
 *   D-pad up / down                flywheel speed +/- 25
 */
@TeleOp(name = "4b - Pedro TeleOp (Advanced)", group = "Competition")
public class PedroTeleOpAdvanced extends OpMode {

    // ---- Pedro drives the wheels ----
    private Follower follower;

    // ---- Mechanisms ----
    private DcMotorEx launcher;
    private DcMotor intake;
    private CRServo leftIntakeServo;
    private CRServo rightIntakeServo;
    private CRServo windmillServo;

    // ---- Launcher and shot detection (the same numbers as the advanced autonomous) ----
    private double launcherTargetVelocity = 1250;
    private static final PIDFCoefficients LAUNCHER_PIDF = new PIDFCoefficients(40, 0, 0, 12.5);
    private static final double SHOT_DIP = 30;         // a dip this big means a ball left
    private static final double FEED_TIMEOUT = 0.75;   // no dip in this long = no ball came out
    private static final double UNJAM_SECONDS = 0.25;

    // The shooter's own little state machine.
    private enum ShooterState { WAITING_FOR_SPEED, FEEDING, UNJAMMING, EMPTY }
    private ShooterState shooterState = ShooterState.WAITING_FOR_SPEED;
    private final ElapsedTime shooterTimer = new ElapsedTime();
    private double velocityBeforeFeed = 0;
    private boolean triedUnjam = false;
    private int ballsShot = 0;
    private boolean wasReady = false;   // so the "ready" rumble only happens once per spin-up

    // ---- Driving ----
    private static final double SLOW_MODE_SPEED = 0.4;
    // How much weaker strafing is than driving forward (T2 / T3 in "0 - Calibration Test").
    private static final double STRAFE_MULTIPLIER = 1.25;
    private boolean fieldCentric = true;
    private boolean blueAlliance = true;
    private boolean flywheelOn = false;
    private boolean positionKnown = false;

    // AIM LOCK: a PID controller that turns the robot toward a heading.
    // P turns harder the further off we are; D (fed the turn speed) stops it overshooting.
    private final PIDController aimController = Controller.pid(1.5, 0, 0.1);

    // DRIVE TO SHOT
    private boolean autoDriving = false;
    private final ElapsedTime autoDriveTimer = new ElapsedTime();
    private static final double AUTO_DRIVE_TIMEOUT = 4.0;

    // END OF MATCH reminder
    private final ElapsedTime matchTimer = new ElapsedTime();
    private static final double TELEOP_SECONDS = 120;
    private boolean parkReminderDone = false;

    // ---- Places on the field, for BLUE. See makeFactory() for RED. ----
    // The two shooting spots from the autonomous: one on each side of our HIVE.
    // The two cell openings they aim at: the FAR cell and the AUDIENCE cell.

    /** Blue poses as they are, red ones turned 180 degrees around the middle of the field. */
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

    /** True when the robot is on the FAR side of the field (y bigger than the middle). */
    private boolean onFarSide() {
        return follower.pose().y() > 72;
    }

    // ============================================================================================
    // INIT
    // ============================================================================================

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);

        // Start from where the advanced autonomous left off, if it ran.
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
        drive();
        runIntakeAndShooter();
        endOfMatchReminder();

        follower.update();   // every loop: read the Pinpoint, drive the wheels

        Pose pose = follower.pose();
        telemetry.addData("Drive", "%s%s%s", fieldCentric ? "FIELD-centric" : "ROBOT-centric",
                autoDriving ? "  >> DRIVING TO SHOT <<" : "",
                gamepad1.left_bumper ? "  >> AIM LOCK <<" : "");
        telemetry.addData("Alliance", blueAlliance ? "BLUE" : "RED");
        telemetry.addData("Position", "x %.1f  y %.1f  heading %.0f%s", pose.x(), pose.y(),
                Math.toDegrees(pose.heading()), positionKnown ? "" : "  (UNKNOWN - d-pad left)");
        telemetry.addData("Flywheel", "%.0f / %.0f ticks/s", launcher.getVelocity(), launcherTargetVelocity);
        telemetry.addData("Shooter", "%s   shots: %d", shooterState, ballsShot);
        telemetry.addData("Time left", "%.0f s", TELEOP_SECONDS - matchTimer.seconds());
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
            // The driver says "the robot is on the autonomous start spot right now".
            follower.setPose(startPose());
            positionKnown = true;
        }

        // Sticks. Pedro: + strafe is LEFT and + turn is COUNTER-CLOCKWISE, so both x's flip.
        double forward = -gamepad1.left_stick_y;
        double left    = -gamepad1.left_stick_x;
        double turn    = -gamepad1.right_stick_x;
        boolean sticksMoved = Math.abs(forward) > 0.2 || Math.abs(left) > 0.2 || Math.abs(turn) > 0.2;

        // ---- DRIVE TO SHOT: Pedro is driving, not the driver ----
        if (gamepad1.dpadUpWasPressed() && positionKnown) {
            startDriveToShot();
        }
        if (autoDriving) {
            boolean arrived = !follower.isBusy();
            boolean tookTooLong = autoDriveTimer.seconds() > AUTO_DRIVE_TIMEOUT;
            if (sticksMoved || arrived || tookTooLong) {
                autoDriving = false;   // hand control back to the driver
            } else {
                return;                // leave the wheels to Pedro this loop
            }
        }

        // ---- Normal driving ----
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

        // Mecanum wheels strafe weaker than they drive forward; make up the difference.
        powers = new DrivePowers(powers.forward(), powers.strafe() * STRAFE_MULTIPLIER, powers.turn());

        // ---- AIM LOCK: replace the driver's turning with turning toward our cell ----
        if (gamepad1.left_bumper && positionKnown) {
            powers = ManualDrive.headingLock(follower, aimController, powers, headingToOurCell());
        } else {
            aimController.reset();   // start fresh next time, so old errors do not kick
        }

        follower.manual(powers);
    }

    /** Pedro drives to the shooting spot on this side of the HIVE. */
    private void startDriveToShot() {
        Pose here = follower.pose();
        Pose target = onFarSide() ? farShotPose() : nearShotPose();
        follower.follow(line(here, target).linear(here, target));
        autoDriving = true;
        autoDriveTimer.reset();
    }

    /**
     * The heading (radians) that points the robot at our HIVE's cell on THIS side of the field.
     * That is the one to shoot at: at the start the far cell is up, and after the HIVE tips the
     * audience cell is - and the shooting spots are on the matching sides.
     */
    private double headingToOurCell() {
        Pose cell = onFarSide() ? farCell() : audienceCell();
        Pose robot = follower.pose();
        return Math.atan2(cell.y() - robot.y(), cell.x() - robot.x());
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
            // A fresh press always starts a fresh try, even after EMPTY.
            shooterState = ShooterState.WAITING_FOR_SPEED;
            triedUnjam = false;
            shooterTimer.reset();
        }

        // ---- Flywheel ----
        if (flywheelOn || shooting) {
            launcher.setVelocity(launcherTargetVelocity);
        } else {
            launcher.setVelocity(0);
        }
        double velocity = launcher.getVelocity();

        // Buzz gamepad 2 once, the moment the flywheel gets up to speed.
        boolean ready = (flywheelOn || shooting) && velocity >= launcherTargetVelocity;
        if (ready && !wasReady) gamepad2.rumble(200);
        wasReady = ready;

        // ---- Windmill ----
        if (shooting) {
            runShooter(velocity);
        } else if (gamepad2.left_bumper) {
            windmillServo.setPower(-1.0);   // driver clearing a jam by hand
        } else {
            windmillServo.setPower(0);
        }

        // ---- Intake ----
        double intakePower = gamepad2.right_trigger - gamepad2.left_trigger;
        intake.setPower(intakePower);
        leftIntakeServo.setPower(intakePower);
        rightIntakeServo.setPower(intakePower);
    }

    /** One ball at a time, using the flywheel dip to see each ball leave. */
    private void runShooter(double velocity) {
        switch (shooterState) {
            case WAITING_FOR_SPEED:
                windmillServo.setPower(0);
                if (velocity >= launcherTargetVelocity) {
                    velocityBeforeFeed = velocity;
                    shooterState = ShooterState.FEEDING;
                    shooterTimer.reset();
                }
                break;

            case FEEDING:
                windmillServo.setPower(1.0);
                if (velocity < velocityBeforeFeed - SHOT_DIP) {
                    // The dip: a ball just left.
                    windmillServo.setPower(0);
                    ballsShot++;
                    triedUnjam = false;
                    shooterState = ShooterState.WAITING_FOR_SPEED;
                } else if (shooterTimer.seconds() > FEED_TIMEOUT) {
                    windmillServo.setPower(0);
                    if (!triedUnjam) {
                        triedUnjam = true;
                        shooterState = ShooterState.UNJAMMING;
                        shooterTimer.reset();
                    } else {
                        shooterState = ShooterState.EMPTY;
                        gamepad2.rumbleBlips(2);   // tell the operator
                    }
                }
                break;

            case UNJAMMING:
                windmillServo.setPower(-1.0);
                if (shooterTimer.seconds() > UNJAM_SECONDS) {
                    windmillServo.setPower(0);
                    shooterState = ShooterState.WAITING_FOR_SPEED;
                }
                break;

            case EMPTY:
                // Nothing to shoot. Wait for the operator to let go and press again.
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
