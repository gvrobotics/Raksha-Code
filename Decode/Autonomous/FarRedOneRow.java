package org.firstinspires.ftc.teamcode.Auto;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;
import com.bylazar.telemetry.PanelsTelemetry;

@Autonomous
public class FarRedOneRow extends OpMode {
    private TelemetryManager panelsTelemetry;
    private Follower follower;

    // State Machine variables
    private Timer pathTimer, opModeTimer, intakeTimer;
    private boolean firstRowPathStarted = false;

    // ---------- FLYWHEEL SETUP ----------
    private Flywheel shooter = new Flywheel();
    private boolean shotsTriggered = false;

    public enum PathState {
        // START POSITION_END POSITION
        // DRIVE -> MOVEMENT STATE
        // SHOOT -> ATTEMPT TO SCORE ARTIFACT
        DRIVE_STARTPOS_SHOOT_POS,
        SHOOT_PRELOAD,
        //------FIRST ROW------
        DRIVE_SHOOTPOS_ROW_START,
        DRIVE_ROW_START_TO_END,
        DRIVE_ROW_ENDPOS_SHOOTPOS,
        SHOOT_ROW,
        DRIVE_SHOOT_ENDPOS;
    }
    PathState pathState;

    private final Pose startPose = new Pose(56.23645320197044, 7.763546798029557, Math.toRadians(90));
    private final Pose shootPose = new Pose(56.23645320197044, 10.137438423645317, Math.toRadians(58));
    //private final Pose secondRowShootPose = new Pose(83.9116117850953, 135.63778162911612, Math.toRadians(0));
    private final Pose rowStart = new Pose(101.17134661191317, 35.86354679802956, Math.toRadians(0));
    private final Pose rowEnd = new Pose(129.00346620450608, 35.86354679802956, Math.toRadians(0));
    private final Pose endPose = new Pose(56.23645320197044, 36.47290640394089, Math.toRadians(180));
    private PathChain driveStartPosShootPos, driveShootPosRowStartPose, driveRowStartToEndPose, driveRowtoShootPose,  driveShootPosEndPose;

    public void buildPaths() {
        // Put in coordinates for starting pose -> ending pose
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(),shootPose.getHeading())
                .build();
        driveShootPosRowStartPose = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, rowStart))
                .setLinearHeadingInterpolation(shootPose.getHeading(),rowStart.getHeading())
                .build();
        driveRowStartToEndPose = follower.pathBuilder()
                .addPath(new BezierLine(rowStart, rowEnd))
                .setLinearHeadingInterpolation(rowStart.getHeading(),rowEnd.getHeading())
                .build();
        driveRowtoShootPose = follower.pathBuilder()
                .addPath(new BezierLine(rowEnd, shootPose))
                .setLinearHeadingInterpolation(rowEnd.getHeading(), shootPose.getHeading())
                .build();
        driveShootPosEndPose = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, endPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), endPose.getHeading())
                .build();

    }

    public void statePathUpdate(){
        switch(pathState) {
            case DRIVE_STARTPOS_SHOOT_POS:
                follower.followPath(driveStartPosShootPos, true);
                shooter.fly1.setVelocity(1250);
                shooter.fly2.setVelocity(1250);
                setPathState(PathState.SHOOT_PRELOAD); //reset timer & make new state
                break;

            case SHOOT_PRELOAD:
                // Check if follower done it's path
                if (!follower.isBusy()) {
                    // requested shots yet
                    if (!shotsTriggered) {
                        shooter.fireShots(3);
                        shotsTriggered = true;
                    }
                    else if (shotsTriggered && !shooter.isBusy()) {
                        // shots are done free to transition
                        follower.followPath(driveShootPosRowStartPose, true);
                        setPathState(PathState.DRIVE_SHOOTPOS_ROW_START);
                    }
                }
                break;

            case DRIVE_SHOOTPOS_ROW_START:
                follower.followPath(driveShootPosRowStartPose, true);
                shooter.intake1.setPower(0.7);
                shooter.intake2.setPower(0.7);
                setPathState(PathState.DRIVE_ROW_START_TO_END);
                intakeTimer.resetTimer();
                break;

            case DRIVE_ROW_START_TO_END:
                if (!firstRowPathStarted) {
                    follower.followPath(driveRowStartToEndPose, true);
                    firstRowPathStarted = true;
                }

                if (!follower.isBusy()) {
                    shooter.intake1.setPower(0);
                    shooter.intake2.setPower(0);
                    firstRowPathStarted = false;
                    setPathState(PathState.DRIVE_ROW_ENDPOS_SHOOTPOS);
                }
                break;

            case DRIVE_ROW_ENDPOS_SHOOTPOS:
                follower.followPath(driveRowtoShootPose, true);
                shooter.fly1.setVelocity(1300);
                shooter.fly2.setVelocity(1300);
                setPathState(PathState.SHOOT_ROW);
                break;

            case SHOOT_ROW:
                // Check if follower done it's path
                if (!follower.isBusy()) {
                    // requested shots yet
                    if (!shotsTriggered) {
                        shooter.fireShots(3);
                        shotsTriggered = true;
                    }
                    else if (shotsTriggered && !shooter.isBusy()) {
                        // shots are done free to transition
                        follower.followPath(driveShootPosEndPose, true);
                        setPathState(PathState.DRIVE_SHOOT_ENDPOS);
                    }
                }
                break;

            case DRIVE_SHOOT_ENDPOS:
                // All done!
                if(!follower.isBusy()) {
                    telemetry.addLine("Done all paths");
                }

            default:
                telemetry.addLine("No State Commanded");
                break;
        }

    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
        firstRowPathStarted = false;
        shotsTriggered = false;
    }

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        pathState = pathState.DRIVE_STARTPOS_SHOOT_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        intakeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        // TODO add in any other init mechanisms
        shooter.init(hardwareMap);


        buildPaths();
        follower.setPose(startPose);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop() {
        follower.update();
        shooter.update();
        statePathUpdate();


        panelsTelemetry.debug("Path state", pathState.toString());
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.debug("Path Time", pathTimer.getElapsedTimeSeconds());

        panelsTelemetry.debug("---- SHOOTER ----");
        panelsTelemetry.debug("Shooter State", shooter.getState());
        panelsTelemetry.debug("Shots Remaining", shooter.getShotsRemaining());
        panelsTelemetry.debug("Flywheel RPM", shooter.getFlywheelRPM());
    }
}
