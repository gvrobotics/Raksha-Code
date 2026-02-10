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
public class RedThreeRowsClose_ArrayCondensing extends OpMode {

    private TelemetryManager panelsTelemetry;
    private Follower follower;

    private Timer pathTimer, opModeTimer, intakeTimer;

    // ---------- SHOOTER ----------
    private Flywheel shooter = new Flywheel();
    private boolean shotsTriggered = false;

    // ---------- ROW CONTROL ----------
    private int currentRow = 0;
    private boolean rowPathStarted = false;

    public enum PathState {
        DRIVE_START_TO_SHOOT,
        SHOOT_PRELOAD,

        DRIVE_TO_ROW_START,
        DRIVE_ROW_START_TO_END,
        DRIVE_ROW_TO_SHOOT,
        SHOOT_ROW,

        DRIVE_TO_END,
        DONE
    }

    private PathState pathState;

    // ---------- POSES ----------
    private final Pose startPose = new Pose(117.54592720970538, 129.2755632582322, Math.toRadians(45));
    private final Pose shootPose = new Pose(92.63570190641249, 99.3275563258232, Math.toRadians(45));
    private final Pose endPose = new Pose(101.2512998266898, 71.06759098786829, Math.toRadians(0));

    // Row arrays
    private final Pose[] rowStarts = {
            new Pose(100.69844020797227, 83.6065857885615, Math.toRadians(0)),
            new Pose(100.69844020797227, 59.82668977469672, Math.toRadians(0)),
            new Pose(101.17134661191317, 35.86354679802956, Math.toRadians(0))
    };

    private final Pose[] rowEnds = {
            new Pose(129.00346620450608, 83.6065857885615, Math.toRadians(0)),
            new Pose(129.00346620450608, 59.82668977469672, Math.toRadians(0)),
            new Pose(129.00346620450608, 35.86354679802956, Math.toRadians(0))
    };

    // ---------- PATHS ----------
    private PathChain driveStartToShoot;
    private PathChain driveToEnd;

    private PathChain[] driveShootToRowStart;
    private PathChain[] driveRowStartToEnd;
    private PathChain[] driveRowToShoot;

    public void buildPaths() {
        driveStartToShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        driveToEnd = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, endPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), endPose.getHeading())
                .build();

        int rows = rowStarts.length;
        driveShootToRowStart = new PathChain[rows];
        driveRowStartToEnd = new PathChain[rows];
        driveRowToShoot = new PathChain[rows];

        for (int i = 0; i < rows; i++) {
            driveShootToRowStart[i] = follower.pathBuilder()
                    .addPath(new BezierLine(shootPose, rowStarts[i]))
                    .setLinearHeadingInterpolation(shootPose.getHeading(), rowStarts[i].getHeading())
                    .build();

            driveRowStartToEnd[i] = follower.pathBuilder()
                    .addPath(new BezierLine(rowStarts[i], rowEnds[i]))
                    .setLinearHeadingInterpolation(rowStarts[i].getHeading(), rowEnds[i].getHeading())
                    .build();

            driveRowToShoot[i] = follower.pathBuilder()
                    .addPath(new BezierLine(rowEnds[i], shootPose))
                    .setLinearHeadingInterpolation(rowEnds[i].getHeading(), shootPose.getHeading())
                    .build();
        }
    }

    public void statePathUpdate() {
        switch (pathState) {

            case DRIVE_START_TO_SHOOT:
                follower.followPath(driveStartToShoot, true);
                shooter.fly1.setVelocity(840);
                shooter.fly2.setVelocity(840);
                setPathState(PathState.SHOOT_PRELOAD);
                break;

            case SHOOT_PRELOAD:
                if (!follower.isBusy()) {
                    if (!shotsTriggered) {
                        shooter.fireShots(3);
                        shotsTriggered = true;
                    } else if (!shooter.isBusy()) {
                        setPathState(PathState.DRIVE_TO_ROW_START);
                    }
                }
                break;

            case DRIVE_TO_ROW_START:
                follower.followPath(driveShootToRowStart[currentRow], true);
                shooter.intake1.setPower(0.7);
                shooter.intake2.setPower(0.7);
                intakeTimer.resetTimer();
                setPathState(PathState.DRIVE_ROW_START_TO_END);
                break;

            case DRIVE_ROW_START_TO_END:
                if (!rowPathStarted) {
                    follower.followPath(driveRowStartToEnd[currentRow], true);
                    rowPathStarted = true;
                }

                if (!follower.isBusy()) {
                    shooter.intake1.setPower(0);
                    shooter.intake2.setPower(0);
                    rowPathStarted = false;
                    setPathState(PathState.DRIVE_ROW_TO_SHOOT);
                }
                break;

            case DRIVE_ROW_TO_SHOOT:
                follower.followPath(driveRowToShoot[currentRow], true);
                shooter.fly1.setVelocity(1300);
                shooter.fly2.setVelocity(1300);
                setPathState(PathState.SHOOT_ROW);
                break;

            case SHOOT_ROW:
                if (!follower.isBusy()) {
                    if (!shotsTriggered) {
                        shooter.fireShots(3);
                        shotsTriggered = true;
                    } else if (!shooter.isBusy()) {
                        currentRow++;

                        if (currentRow >= rowStarts.length) {
                            follower.followPath(driveToEnd, true);
                            setPathState(PathState.DRIVE_TO_END);
                        } else {
                            setPathState(PathState.DRIVE_TO_ROW_START);
                        }
                    }
                }
                break;

            case DRIVE_TO_END:
                if (!follower.isBusy()) {
                    setPathState(PathState.DONE);
                }
                break;

            case DONE:
                telemetry.addLine("Auto complete");
                break;
        }
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
        rowPathStarted = false;
        shotsTriggered = false;
    }

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        pathTimer = new Timer();
        opModeTimer = new Timer();
        intakeTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        shooter.init(hardwareMap);

        buildPaths();
        follower.setPose(startPose);

        pathState = PathState.DRIVE_START_TO_SHOOT;

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();
        shooter.update();
        statePathUpdate();

        panelsTelemetry.debug("Path state", pathState.toString());
        panelsTelemetry.debug("Row", currentRow);
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
