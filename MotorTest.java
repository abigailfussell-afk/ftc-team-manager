package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/*
 * MOTOR TEST - for the build team.
 *
 * Checks that every motor and servo on the StarterBot is plugged in, named correctly, and spins
 * the right way. Put the robot up on a block so the wheels are off the ground first.
 *
 * GAMEPAD 1
 *   Hold X / Y / A / B   spin ONE drive wheel forward at 30% power:
 *                            X = left front    Y = right front
 *                            A = left back     B = right back
 *                        Each wheel should spin so it would push the robot FORWARD.
 *                        If one spins backward, flip its direction in init() below
 *                        (and in every other OpMode!).
 *
 *   D-pad up / down      pick a device from the list on the screen
 *   Left stick up / down run the picked device (push up = forward)
 *
 * Push the robot around by hand to check the Pinpoint: X goes up when you push it forward,
 * Y goes up when you push it LEFT, and heading goes up when you turn it counter-clockwise.
 */
@TeleOp(name = "1 - Motor Test", group = "Test")
public class MotorTest extends OpMode {

    // ---- The drive motors ----
    private DcMotorEx leftFrontDrive;
    private DcMotorEx rightFrontDrive;
    private DcMotorEx leftBackDrive;
    private DcMotorEx rightBackDrive;

    // ---- The mechanisms ----
    private DcMotorEx launcher;      // the flywheel that throws POLLEN
    private DcMotorEx intake;        // the front roller
    private CRServo leftIntakeServo; // the two corner servos that pull pieces out of corners
    private CRServo rightIntakeServo;
    private CRServo windmillServo;   // feeds pieces up into the launcher

    // ---- Odometry ----
    private GoBildaPinpointDriver pinpoint;

    /*
     * Every device in ONE list, so the d-pad can pick any of them.
     *
     * DcMotor and CRServo are different kinds of device, but both of them ARE a "DcMotorSimple",
     * which means both have setPower(). That is what lets them share one array.
     */
    private String[] deviceNames;
    private DcMotorSimple[] devices;
    private int selectedDevice = 0;

    private static final double WHEEL_TEST_POWER = 0.3;

    @Override
    public void init() {
        leftFrontDrive   = hardwareMap.get(DcMotorEx.class, "left_front_drive");
        rightFrontDrive  = hardwareMap.get(DcMotorEx.class, "right_front_drive");
        leftBackDrive    = hardwareMap.get(DcMotorEx.class, "left_back_drive");
        rightBackDrive   = hardwareMap.get(DcMotorEx.class, "right_back_drive");
        launcher         = hardwareMap.get(DcMotorEx.class, "launcher");
        intake           = hardwareMap.get(DcMotorEx.class, "intake");
        leftIntakeServo  = hardwareMap.get(CRServo.class, "left_intake_servo");
        rightIntakeServo = hardwareMap.get(CRServo.class, "right_intake_servo");
        windmillServo    = hardwareMap.get(CRServo.class, "windmillServo");
        pinpoint         = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // The SAME directions every other OpMode uses. This test is checking these lines.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        windmillServo.setDirection(DcMotorSimple.Direction.REVERSE);

        // Reset every motor's encoder to 0, then let it run on plain power.
        DcMotorEx[] motors = { leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive,
                               launcher, intake };
        for (DcMotorEx motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // Pinpoint: our pod positions in inches (see Constants.java for what these mean).
        pinpoint.setOffsets(-0.25, -6.0, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                                      GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();

        deviceNames = new String[] { "left_front_drive", "right_front_drive", "left_back_drive",
                                     "right_back_drive", "launcher", "intake",
                                     "left_intake_servo", "right_intake_servo", "windmillServo" };
        devices = new DcMotorSimple[] { leftFrontDrive, rightFrontDrive, leftBackDrive,
                                        rightBackDrive, launcher, intake,
                                        leftIntakeServo, rightIntakeServo, windmillServo };

        telemetry.addData("Status", "Ready. Wheels off the ground, then press START.");
    }

    @Override
    public void loop() {
        // ---- Pick a device with the d-pad ----
        // "WasPressed" is true for ONE loop when the button goes down, so holding the
        // button moves the selection once instead of fifty times a second.
        if (gamepad1.dpadDownWasPressed()) {
            selectedDevice = selectedDevice + 1;
            if (selectedDevice >= devices.length) selectedDevice = 0;  // wrap to the top
        }
        if (gamepad1.dpadUpWasPressed()) {
            selectedDevice = selectedDevice - 1;
            if (selectedDevice < 0) selectedDevice = devices.length - 1;  // wrap to the bottom
        }

        // ---- Work out a power for every device, starting from "off" ----
        double[] powers = new double[devices.length];

        // The picked device follows the left stick. Pushing a stick UP gives a NEGATIVE
        // number, so we flip it to make "up" mean forward.
        powers[selectedDevice] = -gamepad1.left_stick_y;

        // The wheel buttons. Indexes 0-3 in the list are the four drive motors.
        if (gamepad1.x) powers[0] = WHEEL_TEST_POWER;  // left front
        if (gamepad1.y) powers[1] = WHEEL_TEST_POWER;  // right front
        if (gamepad1.a) powers[2] = WHEEL_TEST_POWER;  // left back
        if (gamepad1.b) powers[3] = WHEEL_TEST_POWER;  // right back

        // Send each power to its device ONCE per loop.
        for (int i = 0; i < devices.length; i++) {
            devices[i].setPower(powers[i]);
        }

        // ---- Show everything on the Driver Hub ----
        pinpoint.update();

        telemetry.addLine("D-pad picks, left stick runs.  X Y A B = one wheel each.");
        telemetry.addLine();
        for (int i = 0; i < devices.length; i++) {
            String arrow = (i == selectedDevice) ? ">> " : "   ";
            telemetry.addData(arrow + deviceNames[i], "power %.2f", powers[i]);
        }

        telemetry.addLine();
        telemetry.addLine("--- Encoders (position / velocity in ticks per second) ---");
        telemetry.addData("left front",  "%d  /  %.0f", leftFrontDrive.getCurrentPosition(), leftFrontDrive.getVelocity());
        telemetry.addData("right front", "%d  /  %.0f", rightFrontDrive.getCurrentPosition(), rightFrontDrive.getVelocity());
        telemetry.addData("left back",   "%d  /  %.0f", leftBackDrive.getCurrentPosition(), leftBackDrive.getVelocity());
        telemetry.addData("right back",  "%d  /  %.0f", rightBackDrive.getCurrentPosition(), rightBackDrive.getVelocity());
        telemetry.addData("launcher",    "%d  /  %.0f", launcher.getCurrentPosition(), launcher.getVelocity());
        telemetry.addData("intake",      "%d  /  %.0f", intake.getCurrentPosition(), intake.getVelocity());

        telemetry.addLine();
        telemetry.addLine("--- Pinpoint (push the robot by hand) ---");
        telemetry.addData("X (in)",        "%.1f", pinpoint.getPosX(DistanceUnit.INCH));
        telemetry.addData("Y (in)",        "%.1f", pinpoint.getPosY(DistanceUnit.INCH));
        telemetry.addData("Heading (deg)", "%.1f", pinpoint.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Raw pods",      "forward %d,  strafe %d", pinpoint.getEncoderX(), pinpoint.getEncoderY());
    }

    @Override
    public void stop() {
        for (DcMotorSimple device : devices) {
            device.setPower(0);
        }
    }
}
