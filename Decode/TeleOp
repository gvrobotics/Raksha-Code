package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class Tele extends OpMode
{
    public DcMotor FR, FL, BR, BL;
    private double powerRY, powerRX, powerLX, powerLY, robotAngle, PowerMultiplier, lf, rb, rf, lb;

    public DcMotor one, two;
    private Servo sp;
    private ElapsedTime timer = new ElapsedTime();

    private Boolean t = false;

    @Override
    public void init()
    {
        FR = hardwareMap.get(DcMotor.class, "FR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        one = hardwareMap.get(DcMotor.class, "1");
        two = hardwareMap.get(DcMotor.class, "2");
        sp = hardwareMap.get(Servo.class, "sp");

        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        one.setDirection(DcMotorSimple.Direction.FORWARD);
        two.setDirection(DcMotorSimple.Direction.REVERSE);
        sp.setDirection(Servo.Direction.FORWARD);

        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        one.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        two.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);
        one.setPower(0);
        two.setPower(0);
        sp.setPosition(0.75);
    }

    @Override
    public void loop()
    {
        powerLX = gamepad1.left_stick_x/2;
        powerLY = gamepad1.left_stick_y/2;
        powerRX = gamepad1.right_stick_x/2;
        powerRY = gamepad1.right_stick_y/2;

        if (gamepad1.y && !t) {
            sp.setPosition(0.95);
            timer.reset();
            t = true;
        }

        if (t && !gamepad1.y && timer.seconds() > 0.2) {
            sp.setPosition(0.75);
            t = false;
        }

        if (gamepad1.x) {
            sp.setPosition(0.95);
            t = false;
        }

        if (gamepad1.dpad_up) {
            one.setPower(0.5);
            two.setPower(0.5);
        } else if (gamepad1.dpad_down) {
            one.setPower(0.7);
            two.setPower(0.7);
        } else if (gamepad1.dpad_left) {
            one.setPower(0.8);
            two.setPower(0.8);
        } else if (gamepad1.dpad_right) {
            one.setPower(0.6);
            two.setPower(0.6);
        } else if (gamepad1.a) {
            one.setPower(0);
            two.setPower(0);
        }

        robotAngle = Math.atan2(powerLX, powerLY);
        telemetry.addData("Robot angle:", robotAngle);
        telemetry.addData("powerRX: ", gamepad1.right_stick_x);
        telemetry.addData("powerRY: ", gamepad1.right_stick_y);
        telemetry.addData("powerLX: ", gamepad1.left_stick_x);
        telemetry.addData("powerLY: ", gamepad1.left_stick_y);

        telemetry.addData("FR: ", FR.getPower());
        telemetry.addData("FL: ", FL.getPower());
        telemetry.addData("BR: ", BR.getPower());
        telemetry.addData("BL: ", BL.getPower());
        telemetry.update();

        telemetry.addData("one: ", one.getPower());
        telemetry.addData("two: ", two.getPower());
        telemetry.addData("move: ", sp.getPosition());
        telemetry.update();


        PowerMultiplier = Math.sqrt((Math.pow(powerLX, 2) + Math.pow(powerLY, 2)));

        lf = (PowerMultiplier*-1*(Math.sin(robotAngle-(Math.PI/4)))) - powerRX;
        rb = (PowerMultiplier*-1*(Math.sin(robotAngle-(Math.PI/4)))) + powerRX;
        lb = (PowerMultiplier*Math.sin(robotAngle+(Math.PI/4))) - powerRX;
        rf = (PowerMultiplier*Math.sin(robotAngle+(Math.PI/4))) + powerRX;

        FR.setPower(rf);
        FL.setPower(lf);
        BR.setPower(rb);
        BL.setPower(lb);

    }
}
