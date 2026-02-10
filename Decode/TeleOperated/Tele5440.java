package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Odo.GoBildaPinpointDriver;

@TeleOp (name = "Tele 5440", group = "A")
public class Tele5440 extends OpMode {
    public DcMotor BR, BL, FR, FL;
    public DcMotorEx fly1, fly2, intake1, intake2;
    public Servo push1, push2, launch;
    GoBildaPinpointDriver odo;

    private Boolean intakeOn = false, prevLB1 = false, currLB1,
            intakeDirection = false, prevLB2 = false, currLB2,
            shooterOn = false, prevRB1 = false, currRB1, flywheelReady = false,
            launchOn = false, prevY2 = false, currY2,
            pushOn = false, prevA2 = false, currA2,
            prevB = false, currB, prevX = false, currX,
            fieldCentricMode = true, prevY = false, currY;
    private int shotsRemaining = 0;
    double TARGET_VELOCITY = 960;
    private double P = 0.212, F = 12.199;
    private double pushUp1 = 0.7, pushDown1 = 0.4, pushUp2 = 0.25, pushDown2 = 0; //push 1 (R) - down is 0.4, up is 0.7 == push 2 (F) - down is 0, up is 0.25

    // State machine for launch sequence
    private enum LaunchState {
        IDLE,
        PUSH_DOWN,
        PUSH_BACK,
        INTAKE_ON
    }

    private LaunchState launchState = LaunchState.IDLE;
    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {
        // Initialize drive motors
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        FL = hardwareMap.get(DcMotor.class, "FL");

        // Initialize mechanism motors
        fly1 = hardwareMap.get(DcMotorEx.class, "f1");
        fly2 = hardwareMap.get(DcMotorEx.class, "f2");
        intake1 = hardwareMap.get(DcMotorEx.class, "i1");
        intake2 = hardwareMap.get(DcMotorEx.class, "i2");

        // Initialize servos
        push1 = hardwareMap.get(Servo.class, "p1");
        push2 = hardwareMap.get(Servo.class, "p2");
        launch = hardwareMap.get(Servo.class, "l");

        // Initialize odometry
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        // Set motor directions
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        fly1.setDirection(DcMotorSimple.Direction.REVERSE);
        fly2.setDirection(DcMotorSimple.Direction.REVERSE);
        intake1.setDirection(DcMotorSimple.Direction.FORWARD);
        intake2.setDirection(DcMotorSimple.Direction.REVERSE);
        push1.setDirection(Servo.Direction.REVERSE);
        push2.setDirection(Servo.Direction.FORWARD);
        launch.setDirection(Servo.Direction.FORWARD);

        // Configure flywheel motors with PIDF
        fly1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        fly2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        fly1.setVelocityPIDFCoefficients(P, 0, 0, F);
        fly2.setVelocityPIDFCoefficients(P, 0, 0, F);

        // Set all motors to brake when power is zero
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fly1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fly2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize all motors to zero power
        BR.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        FL.setPower(0);
        fly1.setPower(0);
        fly2.setPower(0);
        intake1.setVelocity(0);
        intake2.setVelocity(0);
        push1.setPosition(0.4);
        push2.setPosition(0); //push 1 (R) - down is 0.4, up is 0.7 == push 2 (F) - down is 0, up is 0.25
        launch.setPosition(0.5);

        // ===== ODOMETRY CONFIGURATION =====
        odo.setOffsets(-4.33, -3.5, DistanceUnit.INCH);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );

        // Reset and set starting position
        odo.resetPosAndIMU();
        Pose2D startingPosition = new Pose2D(DistanceUnit.MM, -923.925, 1601.47, AngleUnit.RADIANS, 0);
        odo.setPosition(startingPosition);
    }

    @Override
    public void loop() {
        // MUST be called every loop for field-centric to work
        odo.update();

        // ===== TOGGLE DRIVE =====
        prevY = currY;
        currY = gamepad1.y;
        if (currY && !prevY) {
            fieldCentricMode = !fieldCentricMode;
        }

        // ===== DRIVE CONTROL =====
        double y = -gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x * 1.1;
        double rx = gamepad1.right_stick_x;
        double br, bl, fr, fl;

        if (fieldCentricMode) {
            // FIELD-CENTRIC
            // Get robot heading from odometry
            Pose2D pos = odo.getPosition();
            double heading = pos.getHeading(AngleUnit.RADIANS);

            double rotationAngle = (0) - heading;
            double cosAngle = Math.cos(rotationAngle);
            double sinAngle = Math.sin(rotationAngle);

            // Rotate the movement vector by the robot's heading
            double globalStrafe = -y * sinAngle + x * cosAngle;
            double globalForward = y * cosAngle + x * sinAngle;

            // Calculate mecanum wheel powers with field-centric adjustments
            fl = globalForward + globalStrafe + rx;
            fr = globalForward - globalStrafe - rx;
            bl = globalForward - globalStrafe + rx;
            br = globalForward + globalStrafe - rx;
        } else {
            // ROBOT-CENTRIC
            fl = y + x + rx;
            fr = y - x - rx;
            bl = y - x + rx;
            br = y + x - rx;
        }

        // Set motor powers
        BR.setPower(br);
        BL.setPower(bl);
        FR.setPower(fr);
        FL.setPower(fl);

        // ===== INTAKE TOGGLE (Left Bumper) =====
        prevLB1 = currLB1;
        currLB1 = gamepad1.left_bumper;
        if (currLB1 && !prevLB1) {
            intakeOn = !intakeOn;
        }

        double intakePower = 0;

        if (intakeOn) {
            intakePower = intakeDirection ? -0.4 : 0.4;
        }

        intake1.setPower(intakePower);
        intake2.setPower(intakePower);

        // ============= SHOOTER TOGGLE (Right Bumper) ===========
        prevRB1 = currRB1;
        currRB1 = gamepad1.right_bumper;
        if (currRB1 && !prevRB1) {
            shooterOn = !shooterOn;
            if (shooterOn) {
                fly1.setVelocity(1500);
                fly2.setVelocity(1500);
            } else {
                fly1.setVelocity(0);
                fly2.setVelocity(0);
                flywheelReady = false; // reset when turning off
            }
        }

        // ===== FLYWHEEL RUMBLE =====
        if (shooterOn) {
            double avgVelocity =
                    (Math.abs(fly1.getVelocity()) + Math.abs(fly2.getVelocity())) / 2.0;

            if (!flywheelReady && avgVelocity >= TARGET_VELOCITY) {
                gamepad1.rumbleBlips(2); // ONLY gamepad 1
                flywheelReady = true;
            }
        } else {
            flywheelReady = false;
        }

        // ====== LAUNCH SEQUENCE FOR 3 SHOTS (X button) =====
        prevX = currX;
        currX = gamepad1.x;

        if (currX && !prevX && launchState == LaunchState.IDLE) {
            shotsRemaining = 3;

            // Stop intake
            intake1.setPower(0);
            intake2.setPower(0);
            intakeOn = false;

            // Pusher up
            push1.setPosition(pushUp1);
            push2.setPosition(pushUp2);

            timer.reset();
            launchState = LaunchState.PUSH_DOWN;
        }

        // ===== LAUNCH SEQUENCE FOR 1 SHOT (B button) =====
        prevB = currB;
        currB = gamepad1.b;
        // Start launch sequence on B tap if IDLE
        if (currB && !prevB && launchState == LaunchState.IDLE) {
            // Stop intake
            intake1.setVelocity(0);
            intake2.setVelocity(0);
            intakeOn = false;

            // Pusher up
            push1.setPosition(pushUp1);
            push2.setPosition(pushUp2);

            // Start state machine
            timer.reset();
            launchState = LaunchState.PUSH_DOWN;
        }

        // Run launch state machine
        switch (launchState) {
            case PUSH_DOWN:
                if (timer.seconds() >= 0.2) {
                    // Pusher down
                    push1.setPosition(pushDown1);
                    push2.setPosition(pushDown2);
                    timer.reset();
                    launchState = LaunchState.PUSH_BACK;
                }
                break;

            case PUSH_BACK:
                if (timer.seconds() >= 0.2) {
                    // Turn intake on
                    intake1.setVelocity(1000);
                    intake2.setVelocity(1000);
                    timer.reset();
                    launchState = LaunchState.INTAKE_ON;
                }
                break;

            case INTAKE_ON:
                if (timer.seconds() >= 0.5) { // change this value depending on the intake timing issues
                    shotsRemaining--;
                    // Stop intake
                    intake1.setVelocity(0);
                    intake2.setVelocity(0);

                    if (shotsRemaining > 0) {
                        push1.setPosition(pushUp1);
                        push2.setPosition(pushUp2);
                        timer.reset();
                        launchState = LaunchState.PUSH_DOWN;
                    } else {
                        launchState = LaunchState.IDLE;
                    }
                }
                break;

            case IDLE:
            default:
                break;
        }


        // ======== PUSH TOGGLE (A button on 2) =========
        prevA2 = currA2;
        currA2 = gamepad2.a;
        // Only allow A toggle if not in launch sequence
        if (launchState == LaunchState.IDLE && currA2 && !prevA2) {
            pushOn = !pushOn;
            push1.setPosition(pushOn ? pushUp1 : pushDown1); // up - down
            push2.setPosition(pushOn ? pushUp2 : pushDown2);
        }

        // ===== LAUNCH TOGGLE (X button on 2) =====
        prevY2 = currY2;
        currY2 = gamepad2.y;

        if (currY2 && !prevY2) {
            launchOn = !launchOn;
            launch.setPosition(launchOn ? 1 : 0.5);
        }

        // ===== INTAKE DIRECTION TOGGLE (Left bumper on 2) =====
        prevLB2 = currLB2;
        currLB2 = gamepad2.left_bumper;

        if (currLB2 && !prevLB2) {
            intakeDirection = !intakeDirection;
        }

        // ===== TELEMETRY =====
        // Drive mode indicator
        telemetry.addData("DRIVE MODE", fieldCentricMode ? "FIELD-CENTRIC" : "ROBOT-CENTRIC");
        telemetry.addLine();

        telemetry.addLine("========STATE========");
        telemetry.addData("Flywheel", shooterOn ? "ON" : "OFF");
        telemetry.addData("Intake", intakeOn ? "ON" : "OFF");
        telemetry.addData("Intake Direction", intakeDirection ? "REVERSE" : "FORWARD");
        telemetry.addData("Push (a)", pushOn ? "Down" : "Up");
        telemetry.addLine();

        telemetry.addLine("========SHOOTER========");
        telemetry.addData("Shooter Ready", flywheelReady ? "YES" : "NO");
        telemetry.addData("Fly Up", fly1.getVelocity());
        telemetry.addData("Fly Down", fly2.getVelocity());
        telemetry.addData("Fly1 Power", fly1.getPower());
        telemetry.addData("Fly2 Power", fly2.getPower());
        telemetry.addLine();

        telemetry.addLine("========VALUES========");
        telemetry.addData("Push 1", push1.getPosition());
        telemetry.addData("Push 2", push2.getPosition());
        telemetry.addData("Angle", launch.getPosition());
        telemetry.addData("Intake1", intake1.getPower());
        telemetry.addData("Intake2", intake2.getPower());
        telemetry.addLine();

        // Drive motor info
        // Odometry position (only shown in field-centric mode)
        if (fieldCentricMode) {
            Pose2D pos = odo.getPosition();
            telemetry.addData("Robot X", pos.getX(DistanceUnit.MM));
            telemetry.addData("Robot Y", pos.getY(DistanceUnit.MM));
            telemetry.addData("Robot Heading", pos.getHeading(AngleUnit.DEGREES));
            telemetry.addLine();
        }

        telemetry.addData("BR", BR.getPower());
        telemetry.addData("BL", BL.getPower());
        telemetry.addData("FR", FR.getPower());
        telemetry.addData("FL", FL.getPower());
        telemetry.update();
    }
}
