package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.util.Timer;

public class SampleAuton extends OpMode {

    private Follower follower;
    private Timer pathtimer, opModeTimer;

    public enum PathState{
        // START POSITION_END POSITION
        // DRIVE > MOVEMENT STATE
        // SHOOT > ATTEMPT TO SCORE ARTIFACT
        DRIVE_STARTPOS_SHOOT_POS,
        SHOOT_PRELOAD,
        DRIVE_SHOOT_ENDPOS

    }

    PathState pathState;

    private final Pose startPose = new Pose(20.386209877877445, 122.39783853885227, Math.toRadians(138));
    private final Pose shootPose = new Pose(46.415043769588245, 96.90020533880903, Math.toRadians(138));

    private final Pose endPose = new Pose(); //add ending pose here
    private PathChain driveStartPosShootPos, driveShootPosEndPose;

    public void buildPaths() {
        // Put in coordinates for starting pose > ending pose
        driveStartPosShootPos = follower.pathBuilder()
            .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(),shootPose.getHeading())
                .build();
        driveShootPoseEndPos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose,endPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(),endPose.getHeading())
                .build();
    }

    public void statePathUpdate(){
        switch(pathState) {
            case DRIVE_STARTPOS_SHOOT_POS:
                follower.followPath(driveStartPosShootPos, true);
                //pathState = PathState.SHOOT_PRELOAD;
                setPathState(PathState.SHOOT_PRELOAD); //reset timer & make new state
                break;

            case SHOOT_PRELOAD:
                // TODO add logic to flywheel shooter
                // Check if follower done it's path?
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5) {
                    // TODO add logic to flywheel shooter
                    //telemetry.addLine("Done Path 1");
                   // Transition to next state
                    follower.followPath(driveShootPosEndPos, true);
                    setpathState(PathState.DRIVE_SHOOT_ENDPOS);
                }
                break;
            case DRIVE_SHOOT_ENDPOS:
                // All done!
                if(!follower.isBusy()) {
                    telemetry.addLine("Done all paths");
                }

            default:
                telemetry.addLine("No State Command");
                break;
        }

    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathState = pathState.DRIVE_STARTPOS_SHOOT_POS;
        PathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.creatFollower(hardwareMap);
        // TODO add in any other init mechanisms

        buildPaths();
        follower.setPose(startPose);

    }

    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();

        telemetry.addData("path state", pathState.toString());
        telemetry.addDate("x", follower.getPose().getX());
        telemetry.addDate("y", follower.getPose().getY());
        telemetry.addDate("heading", follower.getPose().getHeading());
        telemetry.addDate("Path time", pathTimer.getElapsedTimeSeconds());

    }
}
