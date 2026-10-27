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
 * PEDRO AUTONOMOUS - shoot, collect, shoot, park.
 *
 *   1. Back off the wall and SHOOT the 4 preloaded POLLEN into our HIVE's upward CELL.
 *      The 3rd one TIPS the HIVE, which swings the OTHER cell up.
 *   2. Drive into our corner and PICK UP the 4 POLLEN staged there.
 *   3. Drive to the OTHER SIDE of the HIVE, where the cell that just swung up is facing,
 *      and SHOOT again.
 *   4. PARK in our LOADING ZONE.
 *
 * START POSITION
 *   BLUE: the FAR start, against the far wall, intake facing the audience.
 *   RED:  the AUDIENCE start, against the audience wall, intake facing the far wall.
 *   During INIT, press X for BLUE or B for RED.
 *
 * HOW IT WORKS: A STATE MACHINE
 *   loop() runs about 50 times a second, so it can never "wait" for anything. Instead, the
 *   robot remembers which STEP it is on. Every loop it does a little of that step and checks
 *   whether the step is finished. When it is, it starts the next one.
 *
 * ALL POSES ARE WRITTEN FOR BLUE. For RED, a PoseFactory spins every pose 180 degrees around
 * the middle of the field, because the red side of the field is the blue side turned around.
 * Positions are in INCHES on Pedro's field map: (0, 0) is the corner where the RED wall meets
 * the AUDIENCE wall, x goes toward the BLUE wall and y goes toward the FAR wall. Headings are
 * in DEGREES: 0 faces the blue wall, 90 faces the far wall, counter-clockwise is positive.
 */
@Autonomous(name = "5 - Pedro Auto (Shoot, Collect, Shoot, Park)", group = "Competition")
public class PedroAuto extends OpMode {

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

    // ---- Hardware ----
    private Follower follower;   // Pedro: drives the wheels and knows where the robot is
    private DcMotorEx launcher;
    private DcMotor intake;
    private CRServo leftIntakeServo;
    private CRServo rightIntakeServo;
    private CRServo windmillServo;

    // ---- Launcher settings: the same numbers as the TeleOps ----
    private static final double LAUNCHER_TARGET_VELOCITY = 1250;  // ticks per second
    private static final PIDFCoefficients LAUNCHER_PIDF = new PIDFCoefficients(40, 0, 0, 12.5);
    // How long the windmill runs to push ONE ball. Check this on the real robot: press X in the
    // Flywheel Tuner and watch. If two balls come out, make it shorter; if none do, longer.
    private static final double FEED_ONE_BALL_SECONDS = 0.20;
    private static final double PAUSE_BETWEEN_BALLS = 0.15;    // let the wheel start recovering

    // ---- Timers ----
    private final ElapsedTime matchTimer = new ElapsedTime();  // time since START
    private final ElapsedTime stepTimer = new ElapsedTime();   // time since this step started
    private final ElapsedTime feedTimer = new ElapsedTime();   // used while shooting

    // ---- Shooting ----
    private int ballsShot = 0;
    private boolean feeding = false;

    // ---- Alliance, picked during INIT ----
    private boolean blueAlliance = true;

    // ---- Poses. They are filled in by makePoses() once we know the alliance. ----
    private Pose startPose;
    private Pose firstShotPose;
    private Pose cornerLineUpPose;
    private Pose cornerPickupPose;
    private Pose secondShotPose;
    private Pose parkPose;

    /*
     * Every place the robot goes, written for BLUE.
     *
     * The two SHOT poses were found in the simulator: this launcher has a fixed angle, so the
     * only way to aim the distance is to stand in the right place, about 50 to 55 inches from
     * the cell's opening. Each one FACES the opening of the cell it shoots at, and the heading
     * matters: one degree off is about an inch sideways at the cell. The second shot is on the
     * audience side of the HIVE, because after the tip that is the cell pointing up.
     */
    private void makePoses() {
        PoseFactory p = PoseFactory.degrees();
        if (!blueAlliance) {
            p = p.mirrorAroundPoint(72, 72);  // RED: turn the whole field around
        }

        startPose        = p.of(120, 135, -90);    // FAR start, back against the far wall
        firstShotPose    = p.of(120, 129, -131);   // 6 in off the wall, aimed at the far cell
        cornerLineUpPose = p.of(134, 118, 90);     // below the corner, intake facing it
        cornerPickupPose = p.of(134, 134, 90);     // into the corner, as far as the wall lets us
        secondShotPose   = p.of(120, 18, 133);     // the same shot, on the audience side
        parkPose         = p.of(134, 37, 90);      // inside our LOADING ZONE
    }

    // ---- Paths. Each method returns a fresh path between two poses. ----

    private Path startToFirstShot() {
        return line(startPose, firstShotPose).linear(startPose, firstShotPose);
    }

    private Path firstShotToCorner() {
        return line(firstShotPose, cornerLineUpPose).linear(firstShotPose, cornerLineUpPose);
    }

    /** Slowly, so the intake has time to grab each ball. */
    private Path intoCorner() {
        return line(cornerLineUpPose, cornerPickupPose)
                .constant(cornerPickupPose)
                .with(Constants.foresightConfig.maxPathSpeed.at(0.35));
    }

    private Path cornerToSecondShot() {
        return line(cornerPickupPose, secondShotPose).linear(cornerPickupPose, secondShotPose);
    }

    private Path secondShotToPark() {
        return line(secondShotPose, parkPose).linear(secondShotPose, parkPose);
    }

    // ============================================================================================
    // INIT: runs when the driver presses INIT
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

    /** Runs over and over between INIT and START. The driver picks the alliance here. */
    @Override
    public void init_loop() {
        if (gamepad1.xWasPressed()) blueAlliance = true;
        if (gamepad1.bWasPressed()) blueAlliance = false;

        telemetry.addLine("Press X for BLUE, B for RED.");
        telemetry.addData("Alliance", blueAlliance ? "BLUE - FAR start" : "RED - AUDIENCE start");
    }

    // ============================================================================================
    // START: runs once when the driver presses START
    // ============================================================================================

    @Override
    public void start() {
        makePoses();
        follower.setPose(startPose);  // tell Pedro where we are standing

        matchTimer.reset();
        launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);  // spin up while we drive
        startStep(Step.DRIVE_TO_FIRST_SHOT, startToFirstShot());
    }

    // ============================================================================================
    // LOOP: runs about 50 times a second until the end of autonomous
    // ============================================================================================

    @Override
    public void loop() {
        // Pedro must be updated EVERY loop: it reads the Pinpoint and drives the wheels.
        follower.update();

        // SAFETY NET: parking is worth points, so whatever is happening at 26 seconds,
        // stop it and go park.
        if (matchTimer.seconds() > 26 && step != Step.PARK && step != Step.DONE) {
            stopMechanisms();
            startStep(Step.PARK, pathToPark());
        }

        switch (step) {
            case DRIVE_TO_FIRST_SHOT:
                if (pathFinished(3.0)) {
                    startShooting();
                    step = Step.SHOOT_PRELOADS;
                }
                break;

            case SHOOT_PRELOADS:
                if (shootBalls(4)) {
                    launcher.setVelocity(0);
                    runIntake(1.0);
                    startStep(Step.DRIVE_TO_CORNER, firstShotToCorner());
                }
                break;

            case DRIVE_TO_CORNER:
                if (pathFinished(3.0)) {
                    startStep(Step.PICK_UP, intoCorner());
                }
                break;

            case PICK_UP:
                // The wall stops us before the path ends, so this one finishes on its timeout.
                if (pathFinished(2.5)) {
                    launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);  // spin up while we drive
                    startStep(Step.DRIVE_TO_SECOND_SHOT, cornerToSecondShot());
                }
                break;

            case DRIVE_TO_SECOND_SHOT:
                // Keep the intake running for the first second, to pull in a ball that is
                // only half way in, then turn it off.
                if (stepTimer.seconds() > 1.0) runIntake(0);
                if (pathFinished(5.0)) {
                    startShooting();
                    step = Step.SHOOT_PICKUPS;
                }
                break;

            case SHOOT_PICKUPS:
                if (shootBalls(4)) {
                    stopMechanisms();
                    startStep(Step.PARK, secondShotToPark());
                }
                break;

            case PARK:
                if (pathFinished(4.0)) {
                    step = Step.DONE;
                }
                break;

            case DONE:
                break;
        }

        telemetry.addData("Step", step);
        telemetry.addData("Time", "%.1f s", matchTimer.seconds());
        telemetry.addData("Position", "x %.1f  y %.1f  heading %.0f",
                follower.pose().x(), follower.pose().y(), Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Flywheel", "%.0f / %.0f", launcher.getVelocity(), LAUNCHER_TARGET_VELOCITY);
        telemetry.addData("Balls shot this volley", ballsShot);
    }

    @Override
    public void stop() {
        follower.stop();
        stopMechanisms();
    }

    // ============================================================================================
    // HELPERS
    // ============================================================================================

    /** Moves to a new step and starts following that step's path. */
    private void startStep(Step newStep, Path path) {
        step = newStep;
        follower.follow(path);
        stepTimer.reset();
    }

    /**
     * True when the robot has reached the end of its path and stopped, OR when the step has
     * taken longer than timeoutSeconds. The timeout matters: if the robot gets bumped or
     * hits a wall, it may never get exactly to the end, and without a timeout the
     * autonomous would wait there forever.
     */
    private boolean pathFinished(double timeoutSeconds) {
        return !follower.isBusy() || stepTimer.seconds() > timeoutSeconds;
    }

    /** From wherever we are right now, straight to the park pose. */
    private Path pathToPark() {
        Pose here = follower.pose();
        return line(here, parkPose).linear(here, parkPose);
    }

    private void startShooting() {
        ballsShot = 0;
        feeding = false;
        feedTimer.reset();
    }

    /**
     * Fires balls ONE AT A TIME. Returns true once "count" balls have been fired.
     *
     * Holding the windmill on would empty the robot in a second, but every shot slows the
     * flywheel down, so the 2nd and 3rd balls would leave too slowly and fall short. So we:
     *   wait for full speed -> run the windmill for one ball -> stop -> pause -> repeat.
     */
    private boolean shootBalls(int count) {
        launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);

        if (feeding) {
            // A ball is being pushed in. Stop once it has had time to go.
            if (feedTimer.seconds() > FEED_ONE_BALL_SECONDS) {
                windmillServo.setPower(0);
                feeding = false;
                ballsShot = ballsShot + 1;
                feedTimer.reset();
            }
        } else if (ballsShot < count) {
            // Waiting. Feed the next ball once the wheel is back to FULL speed. (If it never
            // quite gets there, fire anyway after 1.5 seconds rather than waiting forever.)
            boolean pausedLongEnough = feedTimer.seconds() > PAUSE_BETWEEN_BALLS;
            boolean upToSpeed = launcher.getVelocity() >= LAUNCHER_TARGET_VELOCITY;
            if (pausedLongEnough && (upToSpeed || feedTimer.seconds() > 1.5)) {
                windmillServo.setPower(1.0);
                feeding = true;
                feedTimer.reset();
            }
        }

        return ballsShot >= count;
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
