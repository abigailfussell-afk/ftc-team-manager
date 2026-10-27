package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/*
 * STARTERBOT TELEOP - two drivers, plain FTC code (no Pedro Pathing).
 *
 * GAMEPAD 1 - DRIVER
 *   Left stick          drive forward / back and strafe left / right
 *   Right stick (left/right)  turn
 *   Right bumper (hold) slow mode, for lining up a shot
 *   Y                   switch between FIELD-centric and ROBOT-centric driving
 *   X                   we are on the BLUE alliance    B   we are on the RED alliance
 *   A                   reset heading: point the robot straight AWAY from you, then press A
 *
 * GAMEPAD 2 - OPERATOR
 *   Right trigger       intake in          Left trigger   intake out
 *   Y                   flywheel on / off  (spin it up early so it is ready to shoot)
 *   Right bumper (hold) SHOOT: spins up the flywheel and feeds balls once it is up to speed
 *   Left bumper (hold)  run the windmill backward to clear a jam
 *   D-pad up / down     flywheel speed +/- 25  (for shooting from further away / closer)
 *
 * ROBOT-CENTRIC vs FIELD-CENTRIC
 *   Robot-centric: pushing the stick forward drives the way the ROBOT is facing.
 *   Field-centric: pushing the stick forward drives AWAY FROM THE DRIVER, whichever way the
 *   robot is facing. To do that, the code needs to know two things:
 *     1. which way the robot is facing  - the Pinpoint measures this (its "heading")
 *     2. where the driver is standing   - that is what the alliance buttons are for
 */
@TeleOp(name = "3 - StarterBot TeleOp", group = "Competition")
public class StarterBotTeleOp extends OpMode {

    // ---- Hardware ----
    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;
    private DcMotorEx launcher;
    private DcMotor intake;
    private CRServo leftIntakeServo;
    private CRServo rightIntakeServo;
    private CRServo windmillServo;
    private GoBildaPinpointDriver pinpoint;

    // ---- Flywheel settings (find better ones with the Flywheel Tuner) ----
    private double launcherTargetVelocity = 1250;  // ticks per second
    private static final double READY_TO_SHOOT_MARGIN = 50;  // feed once within 50 of target
    private static final PIDFCoefficients LAUNCHER_PIDF = new PIDFCoefficients(40, 0, 0, 12.5);

    private static final double SLOW_MODE_SPEED = 0.4;

    // How much weaker strafing is than driving forward. "0 - Calibration Test" measures it on
    // the real robot (T2 distance / T3 distance) and prints the number to put here.
    private static final double STRAFE_MULTIPLIER = 1.25;

    // ---- Driving state ----
    private boolean fieldCentric = true;
    private boolean blueAlliance = true;
    private boolean flywheelOn = false;

    @Override
    public void init() {
        leftFrontDrive   = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive  = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive    = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive   = hardwareMap.get(DcMotor.class, "right_back_drive");
        launcher         = hardwareMap.get(DcMotorEx.class, "launcher");
        intake           = hardwareMap.get(DcMotor.class, "intake");
        leftIntakeServo  = hardwareMap.get(CRServo.class, "left_intake_servo");
        rightIntakeServo = hardwareMap.get(CRServo.class, "right_intake_servo");
        windmillServo    = hardwareMap.get(CRServo.class, "windmillServo");
        pinpoint         = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // Left motors are mounted facing the other way, so they are reversed.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        windmillServo.setDirection(DcMotorSimple.Direction.REVERSE);

        // BRAKE makes the wheels stop quickly when the sticks are let go.
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // The launcher holds a SPEED, not a power. The hub uses these PIDF numbers to do that.
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, LAUNCHER_PIDF);

        // Tell the Pinpoint where its pods are (see Constants.java for what these mean).
        //
        // We do NOT reset the Pinpoint here. It keeps counting the whole time the robot is on,
        // so if Autonomous just ran, it already knows which way the robot faces on the field.
        // It can still be wrong - the robot was turned off, pushed, or another OpMode changed
        // the heading - so if field-centric drives crooked, point the robot away from you and
        // press A.
        pinpoint.setOffsets(-0.25, -6.0, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                                      GoBildaPinpointDriver.EncoderDirection.FORWARD);

        telemetry.addData("Status", "Ready. Driver: X = Blue, B = Red.");
    }

    @Override
    public void init_loop() {
        // Let the driver pick the alliance before the match starts, too.
        if (gamepad1.xWasPressed()) blueAlliance = true;
        if (gamepad1.bWasPressed()) blueAlliance = false;
        telemetry.addData("Alliance", blueAlliance ? "BLUE" : "RED");
    }

    @Override
    public void loop() {
        pinpoint.update();  // read the Pinpoint once, at the top of every loop

        drive();
        runIntakeAndLauncher();

        telemetry.addData("Drive mode", fieldCentric ? "FIELD-centric" : "ROBOT-centric");
        telemetry.addData("Alliance", blueAlliance ? "BLUE" : "RED");
        telemetry.addData("Heading", "%.0f deg", pinpoint.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Flywheel", "%.0f / %.0f ticks/s  %s", launcher.getVelocity(),
                launcherTargetVelocity, flywheelIsReady() ? "READY" : "");
    }

    // ============================================================================================
    // DRIVING (gamepad 1)
    // ============================================================================================

    private void drive() {
        // ---- Buttons ----
        if (gamepad1.yWasPressed()) fieldCentric = !fieldCentric;
        if (gamepad1.xWasPressed()) blueAlliance = true;
        if (gamepad1.bWasPressed()) blueAlliance = false;
        if (gamepad1.aWasPressed()) {
            // The driver says "the robot is pointing straight away from me right now".
            pinpoint.setHeading(driverForwardHeading(), AngleUnit.RADIANS);
        }

        // ---- Sticks ----
        // Pushing a stick UP gives a NEGATIVE number, so forward is flipped.
        double forward = -gamepad1.left_stick_y;
        double strafe  =  gamepad1.left_stick_x;   // + is right
        double turn    =  gamepad1.right_stick_x;  // + is clockwise

        if (fieldCentric) {
            // How far the robot has turned away from "straight away from the driver".
            // Positive = turned counter-clockwise (to the driver's left).
            double angle = pinpoint.getHeading(AngleUnit.RADIANS) - driverForwardHeading();

            // Rotate the stick direction by the opposite of that angle. Now "stick forward"
            // means "away from the driver" no matter which way the robot is facing.
            double rotatedStrafe  = strafe * Math.cos(-angle) - forward * Math.sin(-angle);
            double rotatedForward = strafe * Math.sin(-angle) + forward * Math.cos(-angle);
            strafe = rotatedStrafe;
            forward = rotatedForward;
        }

        // Mecanum wheels strafe WEAKER than they drive forward, so a diagonal comes out bent
        // toward the robot's nose (field-centric shows it most, about 6 degrees). Boosting the
        // strafe part by the same amount it falls short makes the robot go where it is pointed.
        strafe = strafe * STRAFE_MULTIPLIER;

        if (gamepad1.right_bumper) {
            forward = forward * SLOW_MODE_SPEED;
            strafe  = strafe * SLOW_MODE_SPEED;
            turn    = turn * SLOW_MODE_SPEED;
        }

        mecanumDrive(forward, strafe, turn);
    }

    /**
     * The field heading that points straight away from our driver.
     *
     * Headings are measured like a unit circle: 0 points at the BLUE wall, and counter-clockwise
     * is positive. The red drivers stand at the red wall looking toward blue, so "away from me"
     * is 0. The blue drivers look the other way, so it is 180 degrees (PI radians).
     */
    private double driverForwardHeading() {
        if (blueAlliance) {
            return Math.PI;
        } else {
            return 0;
        }
    }

    /** Turns forward / strafe / turn into four wheel powers. */
    private void mecanumDrive(double forward, double strafe, double turn) {
        double leftFrontPower  = forward + strafe + turn;
        double rightFrontPower = forward - strafe - turn;
        double leftBackPower   = forward - strafe + turn;
        double rightBackPower  = forward + strafe - turn;

        // A motor cannot go faster than 1.0. If any wheel asks for more, slow ALL of them down
        // by the same amount, so the robot still moves in the direction we asked for.
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));
        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    // ============================================================================================
    // INTAKE AND LAUNCHER (gamepad 2)
    // ============================================================================================

    private void runIntakeAndLauncher() {
        // ---- Buttons ----
        if (gamepad2.yWasPressed())        flywheelOn = !flywheelOn;
        if (gamepad2.dpadUpWasPressed())   launcherTargetVelocity += 25;
        if (gamepad2.dpadDownWasPressed()) launcherTargetVelocity -= 25;

        boolean shooting = gamepad2.right_bumper;

        // ---- Flywheel: on when toggled on, or while the shoot button is held ----
        if (flywheelOn || shooting) {
            launcher.setVelocity(launcherTargetVelocity);
        } else {
            launcher.setVelocity(0);
        }

        // ---- Windmill: feed only when the wheel is fast enough to make the shot ----
        if (shooting && flywheelIsReady()) {
            windmillServo.setPower(1.0);
        } else if (gamepad2.left_bumper) {
            windmillServo.setPower(-1.0);   // backward, to clear a jam
        } else {
            windmillServo.setPower(0);
        }

        // ---- Intake: right trigger in, left trigger out ----
        double intakePower = gamepad2.right_trigger - gamepad2.left_trigger;
        intake.setPower(intakePower);
        leftIntakeServo.setPower(intakePower);
        rightIntakeServo.setPower(intakePower);
    }

    private boolean flywheelIsReady() {
        return launcher.getVelocity() > launcherTargetVelocity - READY_TO_SHOOT_MARGIN;
    }

    @Override
    public void stop() {
        mecanumDrive(0, 0, 0);
        launcher.setPower(0);
        intake.setPower(0);
        windmillServo.setPower(0);
        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);
    }
}
