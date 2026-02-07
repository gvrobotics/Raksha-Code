package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Odo.GoBildaPinpointDriver;


@TeleOp
public class AprilTagTest extends OpMode {
    private Limelight3A limelight;
    private IMU imu;
    GoBildaPinpointDriver odo;

    public void init () {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        limelight.pipelineSwitch(0); //TODO: Change pipeline

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));

        // ===== ODOMETRY CONFIGURATION =====
        odo.setOffsets(-4.33, -3.5, DistanceUnit.INCH);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        // Reset and set starting position
        odo.resetPosAndIMU();
        Pose2D startingPosition = new Pose2D(DistanceUnit.MM, -923.925, 1601.47, AngleUnit.RADIANS, 0);
        odo.setPosition(startingPosition);
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop () {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());

        LLResult llresult = limelight.getLatestResult();

        if (llresult != null && llresult.isValid()) {

            Pose3D botPose = llresult.getBotpose_MT2();

            // Get robot position relative to tag (meters by default)
            double x = botPose.getPosition().x;
            double y = botPose.getPosition().y;

            // True distance in XY plane
            double distanceMeters = Math.hypot(x, y);

            // Convert to inches if you want
            double distanceInches = distanceMeters * 39.3701;

            telemetry.addData("Distance (m)", distanceMeters);
            telemetry.addData("Distance (in)", distanceInches);
            telemetry.addData("X", x);
            telemetry.addData("Y", y);
            telemetry.addData("Tx", llresult.getTx());
            telemetry.addData("Ty", llresult.getTy());
            telemetry.addData("Bot Pose", botPose.toString());
        }

        telemetry.update();
    }

}
