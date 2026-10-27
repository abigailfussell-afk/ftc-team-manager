package org.firstinspires.ftc.teamcode;

import static com.pedropathing.api.Paths.line;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;

/*
 * PEDRO AUTONOMOUS - ADVANCED. Same route as "5 - Pedro Auto": shoot, collect, shoot, park.
 * Read that one first. This one adds:
 *
 *   1. SHOT DETECTION. Instead of running the windmill for a set time and HOPING one ball went,
 *      we watch the flywheel. When a ball goes through it, the wheel slows down for a moment
 *      (about 60 ticks/s on this robot). That dip means "a ball just left". So we:
 *          wait for full speed -> run the windmill -> the moment we see a dip, stop -> repeat.
 *      This also COUNTS the balls, so the robot knows when it is empty.
 *
 *   2. EMPTY AND JAM DETECTION. If the windmill runs and no dip comes, either the robot is empty
 *      or a ball is stuck. We run the windmill BACKWARD for a moment to free it and try once
 *      more. If there is still no dip, we decide we are empty and move on.
 *
 *   3. A TIMEOUT ON EVERYTHING. Every step, and every part of shooting, has a limit on how long
 *      it may take. The robot can be bumped, a ball can jam, the battery can sag - and an
 *      autonomous that waits forever for something that will never happen scores nothing.
 *
 *   4. TIME BUDGET. Before starting the second cycle the robot checks there is time to finish
 *      it. If not, it goes straight to park.
 *
 *   5. USING THE PATH'S PROGRESS. follower.completion() says how much of the current path is
 *      done (0 to 1), so we can switch things on and off part way along a path.
 *
 *   6. A SHOT LOG on the screen: when each ball left and how big the dip was. If something goes
 *      wrong on the real robot, this is what to read.
 *
 *   7. HANDING THE POSE TO TELEOP. When autonomous ends, the robot's position is saved in
 *      lastPose, and "4b - Pedro TeleOp (Advanced)" starts from it.
 *
 * During INIT, press X for BLUE (Far start) or B for RED (Audience start).
 */
@Autonomous(name = "5b - Pedro Auto (Advanced)", group = "Competition")
public class PedroAutoAdvanced extends OpMode {

    /**
     * Where the robot was when autonomous ended. "static" means it belongs to the class, not to
     * this one run of the OpMode, so it is still there when the TeleOp starts (as long as the
     * robot stays on). Null until an autonomous has run.
     */
    public static Pose lastPose = null;

    // ---- The steps, in order ----
    private enum Step {
        DRIVE_TO_FIRST_SHOT,
        SHOOT_PRELOADS,
        DRIVE_TO_CORNER,
        PICK_UP,
        DRIVE_TO_SECOND_SHOT,
        SHOOT_PICKUPS,
        PARK,
        DONE
    }
    private Step step = Step.DRIVE_TO_FIRST_SHOT;

    // ---- The shooter has its own little state machine, run inside the SHOOT steps ----
    private enum ShooterState {
        WAITING_FOR_SPEED,   // windmill off, waiting for the flywheel to be back at full speed
        FEEDING,             // windmill on, watching for the dip that says a ball left
        UNJAMMING            // windmill backward for a moment, to free a stuck ball
    }
    private ShooterState shooterState = ShooterState.WAITING_FOR_SPEED;

    // ---- Hardware ----
    private Follower follower;
    private DcMotorEx launcher;
    private DcMotor intake;
    private CRServo leftIntakeServo;
    private CRServo rightIntakeServo;
    private CRServo windmillServo;

    // ---- Launcher settings ----
    private static final double LAUNCHER_TARGET_VELOCITY = 1250;   // ticks per second
    private static final PIDFCoefficients LAUNCHER_PIDF = new PIDFCoefficients(40, 0, 0, 12.5);

    // ---- Shot detection settings. Check these with the Flywheel Tuner's X test. ----
    // A shot slows the wheel by about 60 ticks/s here. Half of that is a safe "yes, a ball left".
    private static final double SHOT_DIP = 30;
    // If no dip comes in this long after the windmill starts, no ball went.
    private static final double FEED_TIMEOUT = 0.75;
    // How long to run the windmill backward to free a stuck ball.
    private static final double UNJAM_SECONDS = 0.25;
    // If the wheel never quite reaches full speed (a tired battery), shoot anyway after this.
    private static final double SPEED_TIMEOUT = 1.5;
    // How close to the shot heading the robot must be before it shoots, in degrees.
    private static final double AIM_TOLERANCE_DEGREES = 2.0;

    // ---- Step time limits, in seconds ----
    private static final double DRIVE_TIMEOUT = 3.0;
    private static final double PICK_UP_TIMEOUT = 2.5;
    private static final double LONG_DRIVE_TIMEOUT = 5.0;
    private static final double VOLLEY_TIMEOUT = 7.0;
    private static final double PARK_TIMEOUT = 4.0;
    // The second cycle (corner, drive, shoot) takes about 9 s. Only start it before this time.
    private static final double LAST_START_FOR_SECOND_CYCLE = 17.0;
    // Whatever is happening at this time, stop and park.
    private static final double PARK_NO_MATTER_WHAT = 26.0;

    // ---- Timers ----
    private final ElapsedTime matchTimer = new ElapsedTime();
    private final ElapsedTime stepTimer = new ElapsedTime();
    private final ElapsedTime shooterTimer = new ElapsedTime();

    // ---- Shooting ----
    private int ballsShot = 0;          // balls that left THIS volley (counted by the dip)
    private int totalBallsShot = 0;
    private boolean triedUnjam = false;
    private double velocityBeforeFeed = 0;
    private String shotLog = "";

    // ---- Alliance and poses ----
    private boolean blueAlliance = true;
    private Pose startPose, firstShotPose, cornerLineUpPose, cornerPickupPose, secondShotPose, parkPose;

    /** Every place the robot goes, written for BLUE. RED is the same, turned around. */
    private void makePoses() {
        PoseFactory p = PoseFactory.degrees();
        if (!blueAlliance) {
            p = p.mirrorAroundPoint(72, 72);
        }
        startPose        = p.of(120, 135, -90);
        firstShotPose    = p.of(120, 129, -131);
        cornerLineUpPose = p.of(134, 118, 90);
        cornerPickupPose = p.of(134, 134, 90);
        secondShotPose   = p.of(120, 18, 133);
        parkPose         = p.of(134, 37, 90);
    }

    private Path startToFirstShot() {
        return line(startPose, firstShotPose).linear(startPose, firstShotPose);
    }

    private Path firstShotToCorner() {
        return line(firstShotPose, cornerLineUpPose).linear(firstShotPose, cornerLineUpPose);
    }

    private Path intoCorner() {
        return line(cornerLineUpPose, cornerPickupPose)
                .constant(cornerPickupPose)
                .with(Constants.foresightConfig.maxPathSpeed.at(0.35));
    }

    private Path cornerToSecondShot() {
        return line(cornerPickupPose, secondShotPose).linear(cornerPickupPose, secondShotPose);
    }

    /** From wherever we are right now, straight to the park pose. */
    private Path toPark() {
        Pose here = follower.pose();
        return line(here, parkPose).linear(here, parkPose);
    }

    // ============================================================================================
    // INIT
    // ============================================================================================

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);

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

        telemetry.addLine("Press X for BLUE, B for RED.");
        telemetry.addData("Alliance", blueAlliance ? "BLUE - FAR start" : "RED - AUDIENCE start");
    }

    @Override
    public void start() {
        makePoses();
        follower.setPose(startPose);
        matchTimer.reset();
        launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);   // spin up while we drive
        startStep(Step.DRIVE_TO_FIRST_SHOT, startToFirstShot());
    }

    // ============================================================================================
    // LOOP
    // ============================================================================================

    @Override
    public void loop() {
        follower.update();

        // SAFETY NET: whatever is happening near the end, stop and go park.
        if (matchTimer.seconds() > PARK_NO_MATTER_WHAT && step != Step.PARK && step != Step.DONE) {
            stopMechanisms();
            shotLog += "| out of time, parking ";
            startStep(Step.PARK, toPark());
        }

        switch (step) {
            case DRIVE_TO_FIRST_SHOT:
                if (pathFinished(DRIVE_TIMEOUT)) {
                    startShooting(Step.SHOOT_PRELOADS);
                }
                break;

            case SHOOT_PRELOADS:
                if (shootBalls(4, firstShotPose)) {
                    // TIME BUDGET: is there time for the second cycle?
                    if (matchTimer.seconds() < LAST_START_FOR_SECOND_CYCLE) {
                        launcher.setVelocity(0);
                        runIntake(1.0);
                        startStep(Step.DRIVE_TO_CORNER, firstShotToCorner());
                    } else {
                        shotLog += "| no time for cycle 2 ";
                        stopMechanisms();
                        startStep(Step.PARK, toPark());
                    }
                }
                break;

            case DRIVE_TO_CORNER:
                if (pathFinished(DRIVE_TIMEOUT)) {
                    startStep(Step.PICK_UP, intoCorner());
                }
                break;

            case PICK_UP:
                // The wall stops us before the path ends, so this one usually finishes on its
                // time limit. That is expected, not an error.
                if (pathFinished(PICK_UP_TIMEOUT)) {
                    startStep(Step.DRIVE_TO_SECOND_SHOT, cornerToSecondShot());
                }
                break;

            case DRIVE_TO_SECOND_SHOT:
                // PATH PROGRESS: halfway there, stop the intake (anything half-in is in by now)
                // and start the flywheel, so it is up to speed when we arrive.
                if (follower.completion() > 0.5) {
                    runIntake(0);
                    launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                }
                if (pathFinished(LONG_DRIVE_TIMEOUT)) {
                    startShooting(Step.SHOOT_PICKUPS);
                }
                break;

            case SHOOT_PICKUPS:
                if (shootBalls(4, secondShotPose)) {
                    stopMechanisms();
                    startStep(Step.PARK, toPark());
                }
                break;

            case PARK:
                if (pathFinished(PARK_TIMEOUT)) {
                    step = Step.DONE;
                }
                break;

            case DONE:
                break;
        }

        lastPose = follower.pose();   // keep it fresh for the TeleOp

        telemetry.addData("Step", "%s  (%.1f s in this step)", step, stepTimer.seconds());
        telemetry.addData("Time", "%.1f s", matchTimer.seconds());
        telemetry.addData("Position", "x %.1f  y %.1f  heading %.0f",
                follower.pose().x(), follower.pose().y(), Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Flywheel", "%.0f / %.0f", launcher.getVelocity(), LAUNCHER_TARGET_VELOCITY);
        telemetry.addData("Shooter", shooterState);
        telemetry.addData("Balls shot", "%d this volley, %d total", ballsShot, totalBallsShot);
        telemetry.addData("Shot log", shotLog);
    }

    @Override
    public void stop() {
        lastPose = follower.pose();
        follower.stop();
        stopMechanisms();
    }

    // ============================================================================================
    // SHOOTING WITH SHOT DETECTION
    // ============================================================================================

    private void startShooting(Step shootStep) {
        step = shootStep;
        stepTimer.reset();
        ballsShot = 0;
        triedUnjam = false;
        shooterState = ShooterState.WAITING_FOR_SPEED;
        shooterTimer.reset();
        shotLog += String.format("| %s: ", shootStep == Step.SHOOT_PRELOADS ? "V1" : "V2");
    }

    /**
     * Shoots up to maxBalls, ONE AT A TIME, using the flywheel's dip to know when each one left.
     * Returns true when this volley is over: all balls shot, the robot is empty, or time is up.
     *
     * Called every loop. Each call does a little bit and returns - it never waits.
     */
    private boolean shootBalls(int maxBalls, Pose aimPose) {
        launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
        double velocity = launcher.getVelocity();

        if (ballsShot >= maxBalls) {
            windmillServo.setPower(0);
            return true;
        }
        if (stepTimer.seconds() > VOLLEY_TIMEOUT) {
            shotLog += "volley timed out ";
            windmillServo.setPower(0);
            return true;
        }

        switch (shooterState) {
            case WAITING_FOR_SPEED:
                windmillServo.setPower(0);
                boolean atSpeed = velocity >= LAUNCHER_TARGET_VELOCITY;
                boolean waitedTooLong = shooterTimer.seconds() > SPEED_TIMEOUT;
                if ((atSpeed || waitedTooLong) && isAimedAt(aimPose)) {
                    // Remember the speed NOW, so we can see it drop when the ball goes through.
                    velocityBeforeFeed = velocity;
                    shooterState = ShooterState.FEEDING;
                    shooterTimer.reset();
                }
                break;

            case FEEDING:
                windmillServo.setPower(1.0);
                if (velocity < velocityBeforeFeed - SHOT_DIP) {
                    // THE DIP: a ball just went through the flywheel.
                    windmillServo.setPower(0);
                    ballsShot++;
                    totalBallsShot++;
                    triedUnjam = false;
                    shotLog += String.format("%.1fs(-%.0f) ", matchTimer.seconds(), velocityBeforeFeed - velocity);
                    shooterState = ShooterState.WAITING_FOR_SPEED;
                    shooterTimer.reset();
                } else if (shooterTimer.seconds() > FEED_TIMEOUT) {
                    // NO DIP: nothing came out. Stuck, or empty?
                    windmillServo.setPower(0);
                    if (!triedUnjam) {
                        triedUnjam = true;
                        shotLog += "unjam ";
                        shooterState = ShooterState.UNJAMMING;
                        shooterTimer.reset();
                    } else {
                        shotLog += "empty ";
                        return true;   // tried twice, nothing came out: we are empty
                    }
                }
                break;

            case UNJAMMING:
                windmillServo.setPower(-1.0);
                if (shooterTimer.seconds() > UNJAM_SECONDS) {
                    windmillServo.setPower(0);
                    shooterState = ShooterState.WAITING_FOR_SPEED;
                    shooterTimer.reset();
                }
                break;
        }
        return false;
    }

    /** True when the robot is pointing within AIM_TOLERANCE_DEGREES of the pose's heading. */
    private boolean isAimedAt(Pose aimPose) {
        // The difference between the two headings, the short way round (-180 to 180 degrees).
        double error = Math.toDegrees(aimPose.heading() - follower.pose().heading());
        while (error > 180) error -= 360;
        while (error < -180) error += 360;
        return Math.abs(error) < AIM_TOLERANCE_DEGREES;
    }

    // ============================================================================================
    // HELPERS
    // ============================================================================================

    private void startStep(Step newStep, Path path) {
        step = newStep;
        follower.follow(path);
        stepTimer.reset();
    }

    /** At the end of the path and stopped, OR this step has used up its time. */
    private boolean pathFinished(double timeoutSeconds) {
        return !follower.isBusy() || stepTimer.seconds() > timeoutSeconds;
    }

    private void runIntake(double power) {
        intake.setPower(power);
        leftIntakeServo.setPower(power);
        rightIntakeServo.setPower(power);
    }

    private void stopMechanisms() {
        launcher.setVelocity(0);
        windmillServo.setPower(0);
        runIntake(0);
    }
}
