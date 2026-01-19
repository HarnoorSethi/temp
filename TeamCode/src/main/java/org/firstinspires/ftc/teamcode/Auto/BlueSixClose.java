package org.firstinspires.ftc.teamcode.Auto;

import android.annotation.SuppressLint;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.*;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.commands.FeedAndShoot;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.util.ScoringGoal;

import java.io.File;

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
    private Timer timer, fullTime;

    private final Pose startPose = new Pose(23, 120, Math.toRadians(180));
    private final ScoringGoal goal = ScoringGoal.BLUE;

    @Override
    public void initialize() {

        drivebase = new Drivebase(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap, telemetry);

        register(drivebase, intake, shooter);

        autoShoot = new FeedAndShoot(shooter, intake);
        autoShoot.fire = false;


        drivebase.setStartingPose(startPose);

        Paths.init(drivebase.follower);

        timer = new Timer();
        fullTime = new Timer();

        pathState = PathState.PRELOAD;
    }

    @Override
    public void runOpMode() throws InterruptedException {


        initialize();
        waitForStart();
        timer.resetTimer();
        fullTime.resetTimer();



        while (opModeIsActive() && !isStopRequested()) {

            if (fullTime.getElapsedTimeSeconds() > 29.5){
                saveFinalPose(drivebase.getPose());
            }

            shooter.periodic();


            autoShoot.updateFeedAndShootDistance(
                    Math.max(
                            Math.min(
                                    drivebase.getPose().distanceFrom(goal.getPose())
                                            + autoShoot.distanceOffset + 5,
                                    130
                            ),
                            10
                    )
            );
            drivebase.follower.update();
            run();
            autoShoot.execute();


            telemetry.addData("distance",
                    drivebase.getPose().distanceFrom(goal.getPose()));

            telemetry.addData("speed", shooter.currentVelocity);
            telemetry.addData("speed target", shooter.targetVelocity);
            telemetry.update();


            updateFSM();
            reset();
        }


    }
    private void saveFinalPose(Pose finalPose) {
        try {
            File poseFile = AppUtil.getInstance().getSettingsFile("finalPose.txt");

            @SuppressLint("DefaultLocale") String data = String.format(
                    "x=%.3f, y=%.3f, heading=%.3f",
                    finalPose.getX(),
                    finalPose.getY(),
                    Math.toDegrees(finalPose.getHeading()) // degrees are easier to read
            );

            ReadWriteFile.writeFile(poseFile, data);

        } catch (Exception e) {
           telemetry.addData("hm", "no pose");
        }
    }

    // ================= FSM LOGIC =================
    private void updateFSM() {
        switch (pathState) {

            case PRELOAD:
                drivebase.follower.followPath(Paths.shootPreload);
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
                    autoShoot.rapidFire = false;
                    drivebase.follower.followPath(Paths.goToBalls);
                    pathState = PathState.GO_BALLS;
                }
                break;

            case GO_BALLS:
                if (!drivebase.follower.isBusy()) {
                    intake.setIntakePower(1);

                    drivebase.follower.followPath(Paths.intakeBalls);
                    pathState = PathState.GO_INTAKE;
                }
                break;

            case GO_INTAKE:
                if (!drivebase.follower.isBusy()) {
                    drivebase.follower.followPath(Paths.shootSpike);
                    pathState = PathState.GO_SHOOT;


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
                    autoShoot.rapidFire = false;

                    drivebase.follower.followPath(Paths.leave);
                    pathState = PathState.LEAVE;
                }
                break;

            case LEAVE:
                intake.setIntakePower(0);
                autoShoot.cancel();
                saveFinalPose(drivebase.getPose());
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
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135 + 180))
                    .build();

            goToBalls = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(62.434, 83.685),
                                    new Pose(55.903, 86.025)))
                    .setLinearHeadingInterpolation(Math.toRadians(135 + 180), Math.toRadians(180))
                    .build();

            intakeBalls = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(55.903, 86.025),
                                    new Pose(22.556, 83.778)))
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            shootSpike = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(22.556, 83.778),
                                    new Pose(62.159, 83.833)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135 + 180))
                    .build();

            leave = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(62.159, 83.833),
                                    new Pose(40.234, 73.545)))
                    .setLinearHeadingInterpolation(Math.toRadians(135 + 180), Math.toRadians(90 + 180))
                    .build();
        }
    }
}
