package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "Auto Shooter", group = "B")
public class AdjustingShooterTest extends OpMode {

    // ===== Hardware =====
    DcMotorEx fly1, fly2, intake1, intake2;
    Servo push1, push2, launch;

    Limelight3A limelight;
    IMU imu;

    // ===== Shooter tuning (EDIT THESE) =====
    double velocitySlope = 25;   // how much velocity increases per inch
    double velocityBase = 900;   // base velocity

    double angleSlope = 0.002;   // servo change per inch
    double angleBase = 0.45;     // base angle

    // ===== Runtime variables =====
    double targetVelocity = 0;
    boolean shooterOn = false;
    boolean intakeOn = false;
    boolean pushOn = false;

    // edge detection
    boolean prevRB, prevLB, prevA;
    boolean prevUp, prevDown, prevLeft, prevRight;

    @Override
    public void init() {

        fly1 = hardwareMap.get(DcMotorEx.class, "f1");
        fly2 = hardwareMap.get(DcMotorEx.class, "f2");
        intake1 = hardwareMap.get(DcMotorEx.class, "i1");
        intake2 = hardwareMap.get(DcMotorEx.class, "i2");

        push1 = hardwareMap.get(Servo.class, "p1");
        push2 = hardwareMap.get(Servo.class, "p2");
        launch = hardwareMap.get(Servo.class, "l");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot orientation =
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                );

        imu.initialize(new IMU.Parameters(orientation));
        limelight.pipelineSwitch(0);
        limelight.start();

        fly1.setDirection(DcMotorSimple.Direction.REVERSE);
        fly2.setDirection(DcMotorSimple.Direction.REVERSE);

        fly1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        fly2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        launch.setPosition(angleBase);
        pushDown();
    }

    @Override
    public void loop() {

        // ===== Update Limelight orientation =====
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(angles.getYaw());

        double distance = getDistance();

        // ===== AUTO AIM =====
        if (distance > 0) {

            targetVelocity = velocityBase + velocitySlope * distance;
            double autoAngle = angleBase + angleSlope * distance;

            launch.setPosition(clamp(autoAngle, 0, 1));
        }

        // ===== Manual fine tuning (D-pad) =====

        if (gamepad1.dpad_right && !prevRight) targetVelocity += 50;
        if (gamepad1.dpad_left && !prevLeft) targetVelocity -= 50;

        if (gamepad1.dpad_up && !prevUp)
            launch.setPosition(clamp(launch.getPosition() + 0.01, 0, 1));

        if (gamepad1.dpad_down && !prevDown)
            launch.setPosition(clamp(launch.getPosition() - 0.01, 0, 1));

        prevRight = gamepad1.dpad_right;
        prevLeft = gamepad1.dpad_left;
        prevUp = gamepad1.dpad_up;
        prevDown = gamepad1.dpad_down;

        // ===== Shooter toggle (RB) =====
        if (gamepad1.right_bumper && !prevRB) shooterOn = !shooterOn;
        prevRB = gamepad1.right_bumper;

        if (shooterOn) {
            fly1.setVelocity(targetVelocity);
            fly2.setVelocity(targetVelocity);
        } else {
            fly1.setVelocity(0);
            fly2.setVelocity(0);
        }

        // ===== Intake toggle (LB) =====
        if (gamepad1.left_bumper && !prevLB) intakeOn = !intakeOn;
        prevLB = gamepad1.left_bumper;

        double intakePower = intakeOn ? 0.6 : 0;
        intake1.setPower(intakePower);
        intake2.setPower(intakePower);

        // ===== Pusher toggle (A) =====
        if (gamepad1.a && !prevA) {
            pushOn = !pushOn;
            if (pushOn) pushUp();
            else pushDown();
        }
        prevA = gamepad1.a;

        // ===== Telemetry =====
        telemetry.addData("Distance (in)", distance);
        telemetry.addData("Target Velocity", targetVelocity);
        telemetry.addData("Actual Velocity", fly1.getVelocity());
        telemetry.addData("Launch Angle", launch.getPosition());
        telemetry.addData("Shooter", shooterOn ? "ON" : "OFF");
        telemetry.update();
    }

    // ===== Distance from Limelight =====
    private double getDistance() {

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            Pose3D pose = result.getBotpose_MT2();
            double x = pose.getPosition().x;
            double y = pose.getPosition().y;
            return Math.hypot(x, y) * 39.3701;
        }

        return -1;
    }

    // ===== Helpers =====
    private void pushUp() {
        push1.setPosition(0.7);
        push2.setPosition(0.25);
    }

    private void pushDown() {
        push1.setPosition(0.4);
        push2.setPosition(0);
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}
