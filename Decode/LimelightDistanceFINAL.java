package org.firstinspires.ftc.teamcode;

// Limelight FTC camera classes
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

// FTC OpMode framework
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// Used for 3D pose data returned by the Limelight
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

// TeleOp annotation so this shows up on the driver station
@TeleOp(name = "AprilTag Distance Limelight")
public class AprilTagDistanceLimelight extends OpMode {

    // Limelight camera object
    private Limelight3A limelight;

    // ===== Mounting constants (EDIT THESE FOR YOUR ROBOT) =====
    // Angle the Limelight is tilted upward from horizontal (degrees)
    private static final double LIME_MOUNT_ANGLE_DEG = 15.0;

    // Height of Limelight lens from the floor (inches)
    private static final double LIME_HEIGHT_IN = 13.0;

    // Height of the AprilTag center from the floor (inches)
    private static final double TAG_HEIGHT_IN = 38.0;

    @Override
    public void init() {

        // Get the Limelight from the hardware map
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Select vision pipeline 0 (must match Limelight configuration)
        limelight.pipelineSwitch(0);

        // Start the Limelight camera stream
        limelight.start();

        // Show status message to drivers
        telemetry.addLine("Limelight Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {

        // Get the latest vision result from Limelight
        LLResult result = limelight.getLatestResult();

        // Check if a valid AprilTag target is detected
        if (result != null && result.isValid()) {

            // ===== METHOD 1: Angle-based distance (trigonometry) =====

            // ty = vertical angle offset from crosshair to target (degrees)
            double ty = result.getTy();

            // Total angle from Limelight to target
            double angleDeg = LIME_MOUNT_ANGLE_DEG + ty;

            // Convert degrees to radians for trig math
            double angleRad = Math.toRadians(angleDeg);

            // Distance calculated using tangent formula
            // distance = height difference / tan(angle)
            double trigDistance =
                    (TAG_HEIGHT_IN - LIME_HEIGHT_IN) / Math.tan(angleRad);

            // ===== METHOD 2: Pose-based distance (3D camera pose) =====

            // Get robot pose relative to the AprilTag
            Pose3D pose = result.getBotpose_MT2();

            // Default value if pose is unavailable
            double poseDistance = -1;

            if (pose != null) {

                // X and Y position of robot relative to tag (meters)
                double x = pose.getPosition().x;
                double y = pose.getPosition().y;

                // Distance = sqrt(x² + y²), converted meters → inches
                poseDistance = Math.hypot(x, y) * 39.3701;
            }

            // ===== Telemetry output to driver station =====

            telemetry.addLine("AprilTag detected!");
            telemetry.addData("Vertical Offset (ty)", ty);
            telemetry.addData("Trig Distance (in)", trigDistance);

            // Only display pose distance if valid
            if (poseDistance > 0) {
                telemetry.addData("Pose Distance (in)", poseDistance);
            }

        } else {

            // No AprilTag detected
            telemetry.addLine("No AprilTag detected");
        }

        // Send telemetry data to driver station
        telemetry.update();
    }
}
