package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

/*
 * ============================================================================
 *  LIMELIGHT BALL CHASE AUTONOMOUS
 * ============================================================================
 *
 *  WHAT THIS PROGRAM DOES
 *  ----------------------
 *  1. SEARCH  - Spin the robot in place until the Limelight sees a yellow ball.
 *  2. CENTER  - Turn until that ball is lined up with the middle of the camera.
 *  3. CHASE   - Turn on the intake and drive forward at the ball, making small
 *               steering corrections, until the ball disappears under the camera.
 *  4. SWEEP   - Keep driving forward a moment longer so the intake grabs it.
 *  5. PAUSE   - Wait 3 seconds.
 *  6. Go back to step 1 and do it all again for the next ball.
 *
 *  WHY IT IS WRITTEN THIS WAY
 *  --------------------------
 *  This is a LinearOpMode, which means the code runs top-to-bottom like a
 *  recipe. That is much easier to read than a state machine when the robot only
 *  ever does one thing at a time. Everything lives in this one file on purpose -
 *  no helper classes, no odometry, no Pedro Pathing. Chasing a ball only needs
 *  the camera and the four drive motors.
 *
 *  BEFORE THIS WILL WORK: SET UP THE LIMELIGHT PIPELINE
 *  ----------------------------------------------------
 *  The Limelight does the image processing, not the robot. You have to teach it
 *  what "yellow ball" looks like ONE TIME using its web interface:
 *
 *    1. Power the robot on. Connect a laptop to the robot's WiFi.
 *    2. In a browser go to  http://limelight.local:5801   (or use the
 *       Limelight's IP address, often 172.29.0.1:5801 over USB).
 *    3. In the pipeline dropdown at the top, pick pipeline 1. Leave pipeline 0
 *       alone - that is the AprilTag pipeline the rest of our code uses.
 *    4. Set the pipeline TYPE to "Color / Retroreflective".
 *    5. Point the camera at a yellow ball on the field. On the "Input" tab turn
 *       the exposure DOWN until the picture looks dark and the ball still glows.
 *       Low exposure is the single biggest trick for reliable color tracking -
 *       it makes the ball stand out and it makes the code immune to gym lights.
 *    6. On the "Thresholding" tab, drag the Hue / Saturation / Value sliders
 *       until ONLY the ball shows up white in the threshold view and everything
 *       else is black. Yellow usually lands near Hue 20-35, with high
 *       Saturation (150-255) and high Value (100-255).
 *    7. On the "Contour Filtering" tab set the area filter so tiny specks of
 *       noise are ignored. Something like 0.05% minimum area is a good start.
 *       You can also set the target area / aspect ratio to prefer round shapes.
 *    8. Walk around the field with a ball and confirm the crosshair sticks to it
 *       and nothing else lights up. Tune until that is true. GARBAGE IN,
 *       GARBAGE OUT - if the pipeline is sloppy the robot will chase a shadow.
 *    9. Make sure the pipeline is SAVED to slot 1 before you unplug.
 *
 *  UNDERSTANDING tx AND ty
 *  -----------------------
 *  The Limelight reports where a target is as two ANGLES, not as X/Y position:
 *     tx = how many degrees LEFT (-) or RIGHT (+) of center the target is.
 *     ty = how many degrees BELOW (-) or ABOVE (+) center the target is.
 *
 *  Our camera is tilted DOWN about 30 degrees, so a ball sitting on the floor
 *  close to the robot appears LOW in the picture (a very negative ty), and a
 *  ball far away appears HIGH in the picture (a ty closer to zero or positive).
 *  That gives us a free way to find the CLOSEST ball: it is the one with the
 *  SMALLEST (most negative) ty.
 *
 *  HOW TO TUNE THIS PROGRAM
 *  ------------------------
 *  Every number you might want to change is a constant at the top of the class.
 *  Change those, not the code below them.
 * ============================================================================
 */
@Autonomous(name = "Limelight Ball Chase", group = "Autonomous")
public class LimelightBallChaseAuto extends LinearOpMode {

    // ========================================================================
    // TUNING CONSTANTS - these are the knobs you turn to change the behavior
    // ========================================================================

    /** Which pipeline slot on the Limelight holds our yellow-ball color pipeline. */
    private static final int BALL_PIPELINE = 1;

    /** How fast the robot spins while hunting for a ball. Higher = faster spin
     *  but the camera is more likely to blur past a ball without seeing it. */
    private static final double SEARCH_TURN_POWER = 0.25;

    /** Flip this to false to make the search spin the other direction. */
    private static final boolean SEARCH_SPINS_CLOCKWISE = true;

    /** How close to dead-center (in degrees) counts as "lined up". Smaller is
     *  more precise but the robot may wobble back and forth forever. */
    private static final double CENTER_TOLERANCE_DEGREES = 3.0;

    /** Proportional gain for turning. Turn power = tx * this number.
     *  Too high = the robot overshoots and oscillates. Too low = it creeps. */
    private static final double TURN_GAIN = 0.020;

    /** The smallest turn power that can actually move the robot. Below this the
     *  motors just buzz and the wheels do not break friction with the floor. */
    private static final double MIN_TURN_POWER = 0.12;

    /** The largest turn power we will ever command while aiming. */
    private static final double MAX_TURN_POWER = 0.35;

    /** How fast the robot drives forward at the ball. */
    private static final double CHASE_DRIVE_POWER = 0.30;

    /** Steering gain used WHILE driving forward. It is gentler than TURN_GAIN
     *  because we only need small corrections, not a full pivot. */
    private static final double CHASE_STEER_GAIN = 0.012;

    /** Power sent to the intake motor. Positive should pull balls IN. If your
     *  intake spits balls out instead, make this negative. */
    private static final double INTAKE_POWER = 1.0;

    /** The ball vanishes from view when it slides underneath the camera. The
     *  detection can also flicker for a fraction of a second. We only believe
     *  the ball is truly gone after it has been missing this long. */
    private static final double LOST_CONFIRM_SECONDS = 0.25;

    /** After the ball disappears under the camera, keep driving forward this
     *  long so the intake actually sweeps it in. Tune this on the field. */
    private static final double SWEEP_SECONDS = 0.75;

    /** How long to sit still after each ball, as required by our routine. */
    private static final double PAUSE_SECONDS = 3.0;

    /** Safety limits. If aiming or chasing takes longer than this, give up and
     *  go back to searching. Without these a bad detection could drive the
     *  robot off the field or leave it spinning in place all match. */
    private static final double CENTER_TIMEOUT_SECONDS = 3.0;
    private static final double CHASE_TIMEOUT_SECONDS  = 4.0;

    /** Ignore blobs smaller than this percent of the image. This throws out
     *  camera noise and yellow things far off the field. */
    private static final double MIN_TARGET_AREA_PERCENT = 0.05;

    // ---- Camera geometry. These are ONLY used for the distance readout on the
    // ---- driver hub. The robot does not steer with them. Measure them on your
    // ---- real robot so the students can see the trig working.
    private static final double CAMERA_HEIGHT_INCHES = 10.0;  // lens height off the floor
    private static final double CAMERA_ANGLE_DEGREES = 30.0;  // how far the camera tilts DOWN
    private static final double BALL_CENTER_HEIGHT_INCHES = 1.5; // 3" ball, so center is 1.5" up

    // ========================================================================
    // HARDWARE - these names must match the robot configuration on the Driver Hub
    // ========================================================================
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor intake;
    private Limelight3A limelight;

    // ========================================================================
    // TRACKING DATA - refreshed every time we look at the camera
    // ========================================================================
    // Instead of creating an extra class to hold "the ball we found", we just
    // store its numbers in these fields. lookForBall() fills them in.
    private boolean ballIsVisible = false;
    private double ballTx = 0.0;    // degrees left(-) / right(+) of center
    private double ballTy = 0.0;    // degrees below(-) / above(+) center
    private double ballArea = 0.0;  // percent of the image the ball fills

    private int ballsCollected = 0;

    // ========================================================================
    // THE MAIN PROGRAM
    // ========================================================================
    @Override
    public void runOpMode() {

        // ---- Grab all the hardware out of the robot configuration ----------
        frontLeft  = hardwareMap.get(DcMotor.class, "leftFront");
        frontRight = hardwareMap.get(DcMotor.class, "rightFront");
        backLeft   = hardwareMap.get(DcMotor.class, "leftRear");
        backRight  = hardwareMap.get(DcMotor.class, "rightRear");
        intake     = hardwareMap.get(DcMotor.class, "intake");

        // Two motors on a robot face opposite directions, so "forward" for the
        // left side is the mirror image of "forward" for the right side. These
        // REVERSE settings make a positive power mean "forward" on all four.
        // (Same settings we use in DriverDanny.java.)
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        // BRAKE makes the robot stop crisply instead of coasting, which matters
        // a lot when we are trying to stop with a ball lined up.
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // RUN_USING_ENCODER lets the motor controller hold a steady speed even
        // if one wheel has more friction than another. It makes the spin and
        // the drive-forward much more consistent.
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // ---- Wake up the Limelight ----------------------------------------
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(BALL_PIPELINE); // use our yellow-ball pipeline
        limelight.setPollRateHz(100);            // ask for new data 100x a second
        limelight.start();                       // begin streaming results

        telemetry.addLine("Ready. Point the camera at a yellow ball to test.");
        telemetry.addData("Pipeline", BALL_PIPELINE);
        telemetry.update();

        // ---- Let the students verify the pipeline BEFORE the match starts ---
        // waitForStart() would just block, so instead we loop and show live
        // camera data. If nothing shows up here, the pipeline needs tuning.
        while (opModeInInit()) {
            lookForBall();
            showTelemetry("WAITING FOR START");
        }

        // =====================================================================
        // MATCH IS LIVE. Repeat the whole routine until the 30 seconds run out
        // or the driver presses stop.
        // =====================================================================
        while (opModeIsActive()) {

            // STEP 1: spin until a ball comes into view.
            boolean found = searchForBall();
            if (!found) break; // only happens if the OpMode was stopped

            // STEP 2: pivot until the ball is centered in front of the intake.
            centerOnBall();

            // STEP 3 & 4: drive over the ball with the intake running.
            chaseAndIntakeBall();

            ballsCollected++;

            // STEP 5: the required 3 second pause.
            stopDriving();
            intake.setPower(0);
            pauseFor(PAUSE_SECONDS);

            // ...and the while loop sends us right back to STEP 1.
        }

        // Always leave the robot safely stopped.
        stopDriving();
        intake.setPower(0);
        limelight.stop();
    }

    // ========================================================================
    // STEP 1: SEARCH
    // ========================================================================
    /**
     * Spins the robot in place until the Limelight reports a yellow ball.
     *
     * @return true if a ball was found, false if the OpMode was stopped first.
     */
    private boolean searchForBall() {
        // Positive turn power spins one way, negative spins the other.
        double turn = SEARCH_SPINS_CLOCKWISE ? SEARCH_TURN_POWER : -SEARCH_TURN_POWER;

        while (opModeIsActive()) {
            lookForBall();

            if (ballIsVisible) {
                stopDriving();   // found one - stop spinning immediately
                return true;
            }

            // forward = 0, strafe = 0, turn = spin in place
            setDrivePower(0, 0, turn);
            showTelemetry("SEARCHING");
        }

        stopDriving();
        return false;
    }

    // ========================================================================
    // STEP 2: CENTER
    // ========================================================================
    /**
     * Pivots the robot until the ball is lined up with the center of the camera
     * (and therefore lined up with the intake, since the camera sits directly
     * above it).
     *
     * This uses PROPORTIONAL CONTROL, the simplest useful control idea in
     * robotics: the further off you are, the harder you correct. When the error
     * shrinks, the correction shrinks with it, so the robot eases into position
     * instead of slamming past it.
     */
    private void centerOnBall() {
        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive() && timer.seconds() < CENTER_TIMEOUT_SECONDS) {
            lookForBall();

            // If the ball flickered out of view, hold still for a moment and
            // look again rather than guessing where it went.
            if (!ballIsVisible) {
                stopDriving();
                showTelemetry("CENTERING - lost sight");
                continue;
            }

            // Are we close enough? Then we are done.
            if (Math.abs(ballTx) <= CENTER_TOLERANCE_DEGREES) {
                stopDriving();
                return;
            }

            // The proportional part: bigger tx means a bigger correction.
            double turn = ballTx * TURN_GAIN;

            // Clamp it so we never command a power that is too weak to move the
            // robot or so strong that we fly past the target.
            turn = clampTurnPower(turn);

            setDrivePower(0, 0, turn);
            showTelemetry("CENTERING");
        }

        // Ran out of time. Stop and let the main loop try again.
        stopDriving();
    }

    // ========================================================================
    // STEPS 3 AND 4: CHASE AND SWEEP
    // ========================================================================
    /**
     * Runs the intake and drives forward at the ball, steering gently to keep it
     * centered. The ball eventually slides underneath the camera and vanishes
     * from the picture - that is our signal that it has reached the intake, so
     * we push forward a little longer to sweep it all the way in.
     */
    private void chaseAndIntakeBall() {
        intake.setPower(INTAKE_POWER);

        ElapsedTime chaseTimer = new ElapsedTime();
        ElapsedTime lostTimer = new ElapsedTime();
        boolean everSawBall = false;

        while (opModeIsActive() && chaseTimer.seconds() < CHASE_TIMEOUT_SECONDS) {
            lookForBall();

            if (ballIsVisible) {
                everSawBall = true;
                lostTimer.reset(); // still visible, so restart the "gone" clock

                // Drive forward while nudging the steering to stay on target.
                // This is the same proportional idea as centerOnBall(), just
                // with a gentler gain so we do not swerve while moving.
                double steer = ballTx * CHASE_STEER_GAIN;
                setDrivePower(CHASE_DRIVE_POWER, 0, steer);
                showTelemetry("CHASING");

            } else {
                // No ball in the picture right now. Keep driving straight - it
                // is probably just underneath us - but start counting.
                setDrivePower(CHASE_DRIVE_POWER, 0, 0);
                showTelemetry("CHASING - ball out of view");

                // Only trust "the ball is gone" if it has been gone long enough
                // that this is not just a one-frame flicker.
                if (everSawBall && lostTimer.seconds() >= LOST_CONFIRM_SECONDS) {
                    break; // the ball is under the robot - go sweep it in
                }
            }
        }

        // STEP 4: final push so the intake fully swallows the ball.
        ElapsedTime sweepTimer = new ElapsedTime();
        while (opModeIsActive() && sweepTimer.seconds() < SWEEP_SECONDS) {
            setDrivePower(CHASE_DRIVE_POWER, 0, 0);
            showTelemetry("SWEEPING IN");
        }

        stopDriving();
    }

    // ========================================================================
    // CAMERA
    // ========================================================================
    /**
     * Asks the Limelight what it can see and stores the CLOSEST yellow ball in
     * ballTx / ballTy / ballArea. Sets ballIsVisible to false if there is
     * nothing worth chasing.
     *
     * Remember: the closest ball is the one lowest in the picture, which means
     * the one with the SMALLEST (most negative) ty.
     */
    private void lookForBall() {
        ballIsVisible = false;

        LLResult result = limelight.getLatestResult();

        // getLatestResult() can hand back null right after startup, before any
        // data has arrived. Always check.
        if (result == null || !result.isValid()) {
            return;
        }

        // A color pipeline can report several blobs at once. Walk the list and
        // keep the lowest one in the picture.
        List<LLResultTypes.ColorResult> blobs = result.getColorResults();

        if (blobs != null && !blobs.isEmpty()) {
            double bestTy = Double.MAX_VALUE;

            for (LLResultTypes.ColorResult blob : blobs) {
                double area = blob.getTargetArea();

                // Skip specks of noise.
                if (area < MIN_TARGET_AREA_PERCENT) {
                    continue;
                }

                double ty = blob.getTargetYDegrees();

                // Lower in the picture = closer to the robot = better.
                if (ty < bestTy) {
                    bestTy = ty;
                    ballTx = blob.getTargetXDegrees();
                    ballTy = ty;
                    ballArea = area;
                    ballIsVisible = true;
                }
            }
        }

        // FALLBACK: some pipeline setups only publish the single primary target
        // through the plain tx / ty values instead of the blob list. If the list
        // was empty but the Limelight still says it has a valid target, use it.
        if (!ballIsVisible && result.getTa() >= MIN_TARGET_AREA_PERCENT) {
            ballTx = result.getTx();
            ballTy = result.getTy();
            ballArea = result.getTa();
            ballIsVisible = true;
        }
    }

    /**
     * BONUS MATH - not used to drive the robot, only shown on the Driver Hub.
     *
     * Because we know how high the camera is and how far down it tilts, a little
     * right-triangle trig turns the ty angle into a distance.
     *
     * First find the total angle the camera is looking down from level. The
     * camera body is already tilted down CAMERA_ANGLE_DEGREES, and ty is negative
     * for a ball low in the picture, so SUBTRACTING ty adds to the tilt:
     *
     *      angle below level = CAMERA_ANGLE_DEGREES - ty
     *
     * Now picture the right triangle: the vertical leg is how much higher the
     * lens is than the middle of the ball, and the horizontal leg is the
     * distance we want. tan(angle) = opposite / adjacent, so:
     *
     *      distance = (camera height - ball height) / tan(angle below level)
     *
     * Sanity check: a closer ball sits lower in the picture, giving a steeper
     * angle, a bigger tangent, and therefore a smaller distance. That matches.
     */
    private double estimateDistanceInches() {
        double angleBelowHorizontal = CAMERA_ANGLE_DEGREES - ballTy;

        // Guard against dividing by zero or by a negative when the "ball" is at
        // or above the horizon, which would be a bad detection anyway.
        if (angleBelowHorizontal <= 1.0) {
            return -1;
        }

        double heightDifference = CAMERA_HEIGHT_INCHES - BALL_CENTER_HEIGHT_INCHES;
        return heightDifference / Math.tan(Math.toRadians(angleBelowHorizontal));
    }

    // ========================================================================
    // DRIVING
    // ========================================================================
    /**
     * The standard mecanum drive equations.
     *
     * Mecanum wheels have angled rollers, so each wheel pushes the robot
     * diagonally. Adding and subtracting the three motions in the right pattern
     * makes the diagonals cancel out and leaves you with the motion you asked
     * for.
     *
     * @param forward  positive drives the robot forward
     * @param strafe   positive slides the robot to the right (this program
     *                 always passes 0, but it is here so you can experiment)
     * @param turn     positive rotates the robot clockwise (to the right)
     */
    private void setDrivePower(double forward, double strafe, double turn) {
        double flPower = forward + strafe + turn;
        double blPower = forward - strafe + turn;
        double frPower = forward - strafe - turn;
        double brPower = forward + strafe - turn;

        // Adding three numbers together can push a power past 1.0, which the
        // motor cannot deliver. If we just chopped it off at 1.0 the wheels
        // would be out of balance and the robot would drift. Instead, if any
        // wheel is over 1.0 we scale ALL FOUR down by the same amount, which
        // keeps the direction of travel correct.
        double biggest = Math.max(Math.abs(flPower), Math.abs(blPower));
        biggest = Math.max(biggest, Math.abs(frPower));
        biggest = Math.max(biggest, Math.abs(brPower));

        if (biggest > 1.0) {
            flPower /= biggest;
            blPower /= biggest;
            frPower /= biggest;
            brPower /= biggest;
        }

        frontLeft.setPower(flPower);
        backLeft.setPower(blPower);
        frontRight.setPower(frPower);
        backRight.setPower(brPower);
    }

    /** Cuts power to all four drive motors. */
    private void stopDriving() {
        setDrivePower(0, 0, 0);
    }

    /**
     * Keeps a turn power inside a useful range: never so small the robot cannot
     * move, never so large it spins out of control. Math.signum() gives us +1 or
     * -1 so we keep turning the same direction we were asked to.
     */
    private double clampTurnPower(double turn) {
        double magnitude = Math.abs(turn);

        if (magnitude < MIN_TURN_POWER) {
            magnitude = MIN_TURN_POWER;
        } else if (magnitude > MAX_TURN_POWER) {
            magnitude = MAX_TURN_POWER;
        }

        return Math.signum(turn) * magnitude;
    }

    // ========================================================================
    // HOUSEKEEPING
    // ========================================================================
    /**
     * Waits the given number of seconds while still updating telemetry, so the
     * Driver Hub does not look frozen. We avoid sleep() here because this keeps
     * the screen live and stays responsive to the stop button.
     */
    private void pauseFor(double seconds) {
        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive() && timer.seconds() < seconds) {
            lookForBall(); // keep the camera data fresh so the screen is useful
            showTelemetry(String.format("PAUSED %.1fs", seconds - timer.seconds()));
        }
    }

    /** One place that prints everything, so every state shows the same info. */
    private void showTelemetry(String state) {
        telemetry.addData("State", state);
        telemetry.addData("Balls collected", ballsCollected);
        telemetry.addData("Ball visible", ballIsVisible);

        if (ballIsVisible) {
            telemetry.addData("tx (left-/right+)", "%.1f deg", ballTx);
            telemetry.addData("ty (down-/up+)", "%.1f deg", ballTy);
            telemetry.addData("area", "%.2f %%", ballArea);
            telemetry.addData("est. distance", "%.1f in", estimateDistanceInches());
        }

        telemetry.update();
    }
}
