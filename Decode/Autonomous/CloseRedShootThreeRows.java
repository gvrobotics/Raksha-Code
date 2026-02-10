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
public class CloseRedShootThreeRows extends OpMode {
    private TelemetryManager panelsTelemetry;
    private Follower follower;

    // State Machine variables
    private Timer pathTimer, opModeTimer, intakeTimer;
    private boolean firstRowPathStarted = false;
    private boolean secondRowPathStarted = false;
    private boolean thirdRowPathStarted = false;

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
        DRIVE_SHOOTPOS_FIRSTROW_START,
        DRIVE_FIRSTROW_START_TO_END,
        DRIVE_FIRST_ENDPOS_SHOOTPOS,
        SHOOT_FIRSTROW,
        //-----SECOND ROW-------
        DRIVE_SHOOT_TO_SECONDROW_START,
        DRIVE_SECONDROW_START_TO_END,
        DRIVE_SECOND_ENDPOS_SHOOTPOS,
        SHOOT_SECONDROW,
        //-----THIRD ROW-------
        DRIVE_SHOOT_TO_THIRDROW_START,
        DRIVE_THIRDROW_START_TO_END,
        DRIVE_THIRD_ENDPOS_SHOOTPOS, //skip if too long, use secondRowShootPose instead
        SHOOT_THIRDROW,
        DRIVE_SHOOT_ENDPOS;
    }
    PathState pathState;

    private final Pose startPose = new Pose(117.54592720970538, 129.2755632582322, Math.toRadians(45));
    private final Pose shootPose = new Pose(92.63570190641249, 99.3275563258232, Math.toRadians(45));
    //private final Pose secondRowShootPose = new Pose(83.9116117850953, 135.63778162911612, Math.toRadians(0));
    private final Pose firstRowStart = new Pose(100.69844020797227, 83.6065857885615, Math.toRadians(0));
    private final Pose firstRowEnd = new Pose(129.00346620450608, 83.6065857885615, Math.toRadians(0));
    private final Pose secondRowStart = new Pose(100.69844020797227, 59.82668977469672, Math.toRadians(0));
    private final Pose secondRowEnd = new Pose(129.00346620450608, 59.82668977469672, Math.toRadians(0));
    private final Pose thirdRowStart = new Pose(101.17134661191317, 35.86354679802956, Math.toRadians(0));
    private final Pose thirdRowEnd = new Pose(129.00346620450608, 35.86354679802956, Math.toRadians(0));
    private final Pose endPose = new Pose(101.2512998266898, 71.06759098786829, Math.toRadians(0));
    private PathChain driveStartPosShootPos, driveShootPosFirstRowStartPose, driveFirstRowStartToEndPose, driveFirstRowtoShootPose, driveShootPosSecondRowStartPose, driveSecondRowStartToEndPose, driveSecondRowtoShootPose, driveShootPosThirdRowStartPose, driveThirdRowStartToEndPose, driveThirdRowtoShootPose, driveShootPosEndPose;

    public void buildPaths() {
        // Put in coordinates for starting pose -> ending pose
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(),shootPose.getHeading())
                .build();
        //---------------------FIRST ROW START----------------------
        driveShootPosFirstRowStartPose = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, firstRowStart))
                .setLinearHeadingInterpolation(shootPose.getHeading(),firstRowStart.getHeading())
                .build();
        driveFirstRowStartToEndPose = follower.pathBuilder()
                .addPath(new BezierLine(firstRowStart, firstRowEnd))
                .setLinearHeadingInterpolation(firstRowStart.getHeading(),firstRowEnd.getHeading())
                .build();
        driveFirstRowtoShootPose = follower.pathBuilder()
                .addPath(new BezierLine(firstRowEnd, shootPose))
                .setLinearHeadingInterpolation(firstRowEnd.getHeading(), shootPose.getHeading())
                .build();
        //---------------------SECOND ROW START------------------------
        driveShootPosSecondRowStartPose = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, secondRowStart))
                .setLinearHeadingInterpolation(shootPose.getHeading(),secondRowStart.getHeading())
                .build();
        driveSecondRowStartToEndPose = follower.pathBuilder()
                .addPath(new BezierLine(secondRowStart, secondRowEnd))
                .setLinearHeadingInterpolation(secondRowStart.getHeading(), secondRowEnd.getHeading())
                .build();
        driveSecondRowtoShootPose = follower.pathBuilder()
                .addPath(new BezierLine(secondRowEnd, shootPose))
                .setLinearHeadingInterpolation(secondRowEnd.getHeading(), shootPose.getHeading())
                .build();
        //---------------------THIRD ROW START------------------------
        driveShootPosThirdRowStartPose = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, thirdRowStart))
                .setLinearHeadingInterpolation(shootPose.getHeading(),thirdRowStart.getHeading())
                .build();
        driveThirdRowStartToEndPose = follower.pathBuilder()
                .addPath(new BezierLine(thirdRowStart, thirdRowEnd))
                .setLinearHeadingInterpolation(thirdRowStart.getHeading(), thirdRowEnd.getHeading())
                .build();
        driveThirdRowtoShootPose = follower.pathBuilder()
                .addPath(new BezierLine(thirdRowEnd, shootPose))
                .setLinearHeadingInterpolation(thirdRowEnd.getHeading(), shootPose.getHeading())
                .build();

        driveShootPosEndPose = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, endPose))
                .setLinearHeadingInterpolation(thirdRowEnd.getHeading(), endPose.getHeading())
                .build();

    }

    public void statePathUpdate(){
        switch(pathState) {
            case DRIVE_STARTPOS_SHOOT_POS:
                follower.followPath(driveStartPosShootPos, true);
                shooter.fly1.setVelocity(840);
                shooter.fly2.setVelocity(840);
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
                        follower.followPath(driveShootPosFirstRowStartPose, true);
                        setPathState(PathState.DRIVE_SHOOTPOS_FIRSTROW_START);
                    }
                }
                break;

//-------------------------------FIRST ROW--------------------------------------
            case DRIVE_SHOOTPOS_FIRSTROW_START:
                follower.followPath(driveShootPosFirstRowStartPose, true);
                shooter.intake1.setPower(0.7);
                shooter.intake2.setPower(0.7);
                setPathState(PathState.DRIVE_FIRSTROW_START_TO_END);
                intakeTimer.resetTimer();
                break;

            case DRIVE_FIRSTROW_START_TO_END:
                if (!firstRowPathStarted) {
                    follower.followPath(driveFirstRowStartToEndPose, true);
                    firstRowPathStarted = true;
                }

                if (!follower.isBusy()) {
                    shooter.intake1.setPower(0);
                    shooter.intake2.setPower(0);
                    firstRowPathStarted = false;
                    setPathState(PathState.DRIVE_FIRST_ENDPOS_SHOOTPOS);
                }
                break;

            case DRIVE_FIRST_ENDPOS_SHOOTPOS:
                follower.followPath(driveFirstRowtoShootPose, true);
                shooter.fly1.setVelocity(1300);
                shooter.fly2.setVelocity(1300);
                setPathState(PathState.SHOOT_FIRSTROW);
                break;

            case SHOOT_FIRSTROW:
                // Check if follower done it's path
                if (!follower.isBusy()) {
                    // requested shots yet
                    if (!shotsTriggered) {
                        shooter.fireShots(3);
                        shotsTriggered = true;
                    }
                    else if (shotsTriggered && !shooter.isBusy()) {
                        // shots are done free to transition
                        follower.followPath(driveShootPosSecondRowStartPose, true);
                        setPathState(PathState.DRIVE_SHOOT_TO_SECONDROW_START);
                    }
                }
                break;
//-------------------------------SECOND ROW--------------------------------------
            case DRIVE_SHOOT_TO_SECONDROW_START:
                follower.followPath(driveShootPosSecondRowStartPose, true);
                shooter.intake1.setPower(0.7);
                shooter.intake2.setPower(0.7);
                setPathState(PathState.DRIVE_SECONDROW_START_TO_END);
                intakeTimer.resetTimer();
                break;

            case DRIVE_SECONDROW_START_TO_END:
                if (!secondRowPathStarted) {
                    follower.followPath(driveSecondRowStartToEndPose, true);
                    secondRowPathStarted = true;
                }

                if (!follower.isBusy()) {
                    shooter.intake1.setPower(0);
                    shooter.intake2.setPower(0);
                    secondRowPathStarted = false;
                    setPathState(PathState.DRIVE_SECOND_ENDPOS_SHOOTPOS);
                }
                break;

            case DRIVE_SECOND_ENDPOS_SHOOTPOS:
                follower.followPath(driveSecondRowtoShootPose, true);
                shooter.fly1.setVelocity(1300);
                shooter.fly2.setVelocity(1300);
                setPathState(PathState.SHOOT_SECONDROW);
                break;

            case SHOOT_SECONDROW:
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
                        setPathState(PathState.DRIVE_SHOOT_TO_THIRDROW_START);
                    }
                }
                break;
//-------------------------------THIRD ROW--------------------------------------
            case DRIVE_SHOOT_TO_THIRDROW_START:
                follower.followPath(driveShootPosThirdRowStartPose, true);
                shooter.intake1.setPower(0.7);
                shooter.intake2.setPower(0.7);
                setPathState(PathState.DRIVE_THIRDROW_START_TO_END);
                intakeTimer.resetTimer();
                break;

            case DRIVE_THIRDROW_START_TO_END:
                if (!thirdRowPathStarted) {
                    follower.followPath(driveThirdRowStartToEndPose, true);
                    thirdRowPathStarted = true;
                }

                if (!follower.isBusy()) {
                    shooter.intake1.setPower(0);
                    shooter.intake2.setPower(0);
                    thirdRowPathStarted = false;
                    setPathState(PathState.DRIVE_THIRD_ENDPOS_SHOOTPOS);
                }
                break;

            case DRIVE_THIRD_ENDPOS_SHOOTPOS:
                follower.followPath(driveThirdRowtoShootPose, true);
                shooter.fly1.setVelocity(1300);
                shooter.fly2.setVelocity(1300);
                setPathState(PathState.SHOOT_THIRDROW);
                break;

            case SHOOT_THIRDROW:
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
        secondRowPathStarted = false;
        thirdRowPathStarted = false;
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
