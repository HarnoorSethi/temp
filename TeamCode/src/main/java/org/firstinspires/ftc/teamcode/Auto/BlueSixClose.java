package org.firstinspires.ftc.teamcode.Auto;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.*;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.FeedAndShoot;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.util.ScoringGoal;

@Autonomous(name = "Blue Six Close (FSM)", group = "Autonomous")
public class BlueSixClose extends CommandOpMode {

    // ================= FSM =================
    private enum PathState {
        PRELOAD, WAIT_PRELOAD, FIRE_PRELOAD,
        GO_BALLS, GO_INTAKE,
        GO_SHOOT, WAIT_SHOOT, FIRE_SHOOT,
        LEAVE
    }

    private PathState pathState;

    // ================= Subsystems =================
    private Drivebase drivebase;
    private Intake intake;
    private Shooter shooter;

    // ================= Commands =================
    private FeedAndShoot autoShoot;

    // ================= Utils =================
    private Timer timer;

    private final Pose startPose = new Pose(72, 8, Math.toRadians(90));
    private final ScoringGoal goal = ScoringGoal.BLUE;

    @Override
    public void initialize() {

        drivebase = new Drivebase(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap, telemetry);

        register(drivebase, intake, shooter);

        autoShoot = new FeedAndShoot(shooter, intake);
        autoShoot.fire = false;
        autoShoot.schedule(); // runs continuously like the other team's autoShoot

        drivebase.setStartingPose(startPose);

        Paths.init(drivebase.follower);

        timer = new Timer();
        pathState = PathState.PRELOAD;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        timer.resetTimer();

        while (opModeIsActive() && !isStopRequested()) {


            autoShoot.updateFeedAndShootDistance(
                    drivebase.getPose().distanceFrom(goal.getPose())
            );

            updateFSM();
        }
    }

    // ================= FSM LOGIC =================
    private void updateFSM() {
        switch (pathState) {

            case PRELOAD:
                drivebase.followPath(Paths.shootPreload);
                pathState = PathState.WAIT_PRELOAD;
                break;

            case WAIT_PRELOAD:
                if (!drivebase.follower.isBusy()) {
                    timer.resetTimer();
                    pathState = PathState.FIRE_PRELOAD;
                }
                break;

            case FIRE_PRELOAD:
                if (timer.getElapsedTimeSeconds() > 0.4)
                    autoShoot.fire = true;

                if (timer.getElapsedTimeSeconds() > 2) {
                    autoShoot.fire = false;
                    drivebase.followPath(Paths.goToBalls);
                    pathState = PathState.GO_BALLS;
                }
                break;

            case GO_BALLS:
                if (!drivebase.follower.isBusy()) {
                    intake.setIntakePower(1);

                    drivebase.followPath(Paths.intakeBalls);
                    pathState = PathState.GO_INTAKE;
                }
                break;

            case GO_INTAKE:
                if (!drivebase.follower.isBusy()) {
                    drivebase.followPath(Paths.shootSpike);
                    pathState = PathState.GO_SHOOT;
                    intake.setIntakePower(0);

                }
                break;

            case GO_SHOOT:
                if (!drivebase.follower.isBusy()) {
                    timer.resetTimer();
                    pathState = PathState.WAIT_SHOOT;
                }
                break;

            case WAIT_SHOOT:
                if (timer.getElapsedTimeSeconds() > 0.4) {
                    autoShoot.fire = true;
                    timer.resetTimer();
                    pathState = PathState.FIRE_SHOOT;
                }
                break;

            case FIRE_SHOOT:
                if (timer.getElapsedTimeSeconds() > 2) {
                    autoShoot.fire = false;
                    drivebase.followPath(Paths.leave);
                    pathState = PathState.LEAVE;
                }
                break;

            case LEAVE:
                intake.setIntakePower(0);
                break;
        }
    }

    // ================= PATH BANK =================
    public static class Paths {
        public static PathChain shootPreload, goToBalls, intakeBalls, shootSpike, leave;

        public static void init(Follower follower) {

            shootPreload = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(23.453, 120.589),
                                    new Pose(62.434, 83.685)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                    .build();

            goToBalls = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(62.434, 83.685),
                                    new Pose(38.903, 84.025)))
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                    .build();

            intakeBalls = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(38.903, 84.025),
                                    new Pose(15.556, 83.778)))
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            shootSpike = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(15.556, 83.778),
                                    new Pose(62.159, 83.833)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                    .build();

            leave = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(62.159, 83.833),
                                    new Pose(25.234, 73.545)))
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(90))
                    .build();
        }
    }
}
