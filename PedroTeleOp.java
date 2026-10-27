package org.firstinspires.ftc.teamcode;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.pedro.Constants;

/*
 * PEDRO TELEOP - the same robot and the same buttons as "3 - StarterBot TeleOp", but the
 * DRIVING goes through Pedro Pathing's Follower.
 *
 * What Pedro does for us here:
 *   - It owns the four drive motors (names and directions live in pedro/Constants.java).
 *   - It reads the Pinpoint every loop, so follower.pose() always says where the robot is,
 *     in inches and radians, on Pedro's field map.
 *   - follower.manual(...) drives robot-centric, and ManualDrive.fieldCentric(...) turns the
 *     sticks into field-centric powers.
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
 *   D-pad up / down     flywheel speed +/- 25
 */
@TeleOp(name = "4 - Pedro TeleOp", group = "Competition")
public class PedroTeleOp extends OpMode {

    // ---- Pedro drives the wheels ----
    private Follower follower;

    // ---- Everything else is ours ----
    private DcMotorEx launcher;
    private DcMotor intake;
    private CRServo leftIntakeServo;
    private CRServo rightIntakeServo;
    private CRServo windmillServo;

    // ---- Flywheel settings (find better ones with the Flywheel Tuner) ----
    private double launcherTargetVelocity = 1250;  // ticks per second
    private static final double READY_TO_SHOOT_MARGIN = 50;
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
        // One line builds the drivetrain, the Pinpoint and the path follower.
        //
        // HEADING: building the Follower recalibrates the Pinpoint's IMU, and that can set the
        // heading back to 0 (facing the blue wall) wherever the robot is really pointing.
        // Field-centric driving is only as good as the heading, so before driving field-centric:
        // point the robot straight away from you and press A.
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

        telemetry.addData("Status", "Ready. Driver: X = Blue, B = Red.");
    }

    @Override
    public void init_loop() {
        if (gamepad1.xWasPressed()) blueAlliance = true;
        if (gamepad1.bWasPressed()) blueAlliance = false;
        telemetry.addData("Alliance", blueAlliance ? "BLUE" : "RED");
        telemetry.addLine("Field-centric: after START, point the robot away from you and press A.");
    }

    @Override
    public void loop() {
        drive();
        runIntakeAndLauncher();

        // Pedro must be updated EVERY loop: this reads the Pinpoint and sends power to the wheels.
        follower.update();

        telemetry.addData("Drive mode", fieldCentric ? "FIELD-centric" : "ROBOT-centric");
        telemetry.addData("Alliance", blueAlliance ? "BLUE" : "RED");
        telemetry.addData("Position", "x %.1f  y %.1f in", follower.pose().x(), follower.pose().y());
        telemetry.addData("Heading", "%.0f deg", Math.toDegrees(follower.pose().heading()));
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
            follower.setHeading(driverForwardHeading());
        }

        // ---- Sticks ----
        // PEDRO'S DIRECTIONS ARE NOT THE GAMEPAD'S. Pedro uses the math directions:
        // + strafe is LEFT and + turn is COUNTER-CLOCKWISE. A stick pushed right gives a
        // POSITIVE x, so both stick x values are flipped. (Forward is flipped as usual,
        // because pushing a stick up gives a negative y.)
        double forward = -gamepad1.left_stick_y;
        double left    = -gamepad1.left_stick_x;
        double turn    = -gamepad1.right_stick_x;

        if (gamepad1.right_bumper) {
            forward = forward * SLOW_MODE_SPEED;
            left    = left * SLOW_MODE_SPEED;
            turn    = turn * SLOW_MODE_SPEED;
        }

        DrivePowers powers;
        if (fieldCentric) {
            // Pedro rotates the sticks for us. We give it how far the robot has turned away
            // from "straight away from the driver".
            double angle = follower.pose().heading() - driverForwardHeading();
            powers = ManualDrive.fieldCentric(forward, left, turn, angle);
        } else {
            powers = new DrivePowers(forward, left, turn);
        }

        // Mecanum wheels strafe WEAKER than they drive forward, so a diagonal comes out bent
        // toward the robot's nose (field-centric shows it most, about 6 degrees). Boosting the
        // strafe part by the same amount it falls short makes the robot go where it is pointed.
        powers = new DrivePowers(powers.forward(), powers.strafe() * STRAFE_MULTIPLIER, powers.turn());

        follower.manual(powers);
    }

    /**
     * The field heading that points straight away from our driver.
     *
     * Pedro measures headings like a unit circle: 0 points at the BLUE wall, and counter-clockwise
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

    // ============================================================================================
    // INTAKE AND LAUNCHER (gamepad 2) - the same code as the StarterBot TeleOp
    // ============================================================================================

    private void runIntakeAndLauncher() {
        if (gamepad2.yWasPressed())        flywheelOn = !flywheelOn;
        if (gamepad2.dpadUpWasPressed())   launcherTargetVelocity += 25;
        if (gamepad2.dpadDownWasPressed()) launcherTargetVelocity -= 25;

        boolean shooting = gamepad2.right_bumper;

        if (flywheelOn || shooting) {
            launcher.setVelocity(launcherTargetVelocity);
        } else {
            launcher.setVelocity(0);
        }

        if (shooting && flywheelIsReady()) {
            windmillServo.setPower(1.0);
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

    private boolean flywheelIsReady() {
        return launcher.getVelocity() > launcherTargetVelocity - READY_TO_SHOOT_MARGIN;
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
