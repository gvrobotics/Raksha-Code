package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class test extends OpMode
{
    public DcMotor one, two;
    private Servo sp;
    private ElapsedTime timer = new ElapsedTime();

    private Boolean t = false;

    @Override
    public void init()
    {
        one = hardwareMap.get(DcMotor.class, "1");
        two = hardwareMap.get(DcMotor.class, "2");
        sp = hardwareMap.get(Servo.class, "sp");

        one.setDirection(DcMotorSimple.Direction.FORWARD);
        two.setDirection(DcMotorSimple.Direction.REVERSE);
        sp.setDirection(Servo.Direction.FORWARD);

        one.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        two.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        one.setPower(0);
        two.setPower(0);
        sp.setPosition(0.75);
    }

    @Override
    public void loop()
    {

        telemetry.addData("one: ", one.getPower());
        telemetry.addData("two: ", two.getPower());
        telemetry.addData("move: ", sp.getPosition());
        telemetry.update();


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
    }
}
