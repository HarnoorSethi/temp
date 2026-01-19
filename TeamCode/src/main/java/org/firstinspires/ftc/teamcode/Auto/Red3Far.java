package org.firstinspires.ftc.teamcode.Auto;

import android.annotation.SuppressLint;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.*;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.commands.FeedAndShoot;
import org.firstinspires.ftc.teamcode.commands.autoAlign;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.util.ScoringGoal;

import java.io.File;

@Autonomous(name = "RED 3 and leave", group = "Autonomous")
public class Red3Far extends CommandOpMode {

    // ================= FSM =================
    private enum PathState {
        PRELOAD,
        WAIT_PRELOAD,
        FIRE_PRELOAD,

        GO_CORNER,
        INTAKE_CORNER,

        GO_SHOOT,
        WAIT_SHOOT,
        FIRE_SHOOT,
        ThirdBall,
        DriveForward,

        LEAVE
    }

    private PathState pathState;

    // ================= Subsystems =================
    private Drivebase drivebase;
    private Intake intake;
    private Shooter shooter;
    private autoAlign AutoAlign;

    // ================= Commands =================
    private FeedAndShoot autoShoot;

    // ================= Utils =================
    private Timer timer, fullTime;
    private Limelight3A ll;

    private final Pose startPose = new Pose(56.000, 8.000, Math.toRadians(90)).mirror();
    private final ScoringGoal goal = ScoringGoal.RED;

    @Override
    public void initialize() {

        drivebase = new Drivebase(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap, telemetry);

        register(drivebase, intake, shooter);

        autoShoot = new FeedAndShoot(shooter, intake);
        autoShoot.fire = false;
        autoShoot.rapidFire = false;
        AutoAlign = new autoAlign(drivebase,ScoringGoal.RED,ll);

        drivebase.setStartingPose(startPose);

        Paths.init(drivebase.follower);

        timer = new Timer();
        fullTime = new Timer();
        pathState = PathState.PRELOAD;
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
                                            + autoShoot.distanceOffset,
                                    130
                            ),
                            10
                    )
            );

            drivebase.follower.update();
            run();
            autoShoot.execute();
            autoShoot.distanceOffset = 38;

            telemetry.addData("Pose", drivebase.getPose());
            telemetry.addData("Shooter Speed", shooter.currentVelocity);
            telemetry.addData("Target Speed", shooter.targetVelocity);
            telemetry.update();

            updateFSM();
            reset();
        }
    }

    // ================= FSM LOGIC =================
    private void updateFSM() {
        switch (pathState) {

            case PRELOAD:
                drivebase.follower.followPath(Paths.preloadShoot);
                pathState = PathState.WAIT_PRELOAD;
                break;

            case WAIT_PRELOAD:
                if (!drivebase.follower.isBusy()) {
                    timer.resetTimer();
                    if (shooter.isReadyToShoot()){
                        pathState = PathState.FIRE_PRELOAD;}
                }
                break;

            case FIRE_PRELOAD:
                if (timer.getElapsedTimeSeconds() > 2.0) {
                    autoShoot.fire = true;
                    autoShoot.rapidFire = true;
                }

                if (timer.getElapsedTimeSeconds() > 6.0) {
                    autoShoot.fire = false;
                    autoShoot.rapidFire = false;
                    drivebase.follower.followPath(Paths.Leave);
                    pathState = PathState.LEAVE;
                }
                break;



            case LEAVE:
                intake.setIntakePower(0);
                saveFinalPose(drivebase.getPose());
                break;
        }
    }

    // ================= PATH BANK =================
    public static class Paths {

        public static PathChain preloadShoot;
        public static PathChain GoToCorner;
        public static PathChain IntakeCorner;
        public static PathChain ShootCorner;
        public static PathChain Leave;
        public static PathChain ThirdBall;
        public static PathChain DriveForward;


        public static void init(Follower follower) {

            preloadShoot = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(56.000, 8.000).mirror(),
                            new Pose(55.821, 16.757).mirror()))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(18 - 90),
                            Math.toRadians(245))
                    .build();

            GoToCorner = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(55.821, 20.757).mirror(),
                            new Pose(10.963, 16.472).mirror()))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(180 - 295),
                            Math.toRadians(180 -210))
                    .build();

            IntakeCorner = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(10.963, 16.472).mirror(),
                            new Pose(9.184, 10.078).mirror()))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(180 - 210),
                            Math.toRadians(180 - 180))
                    .build();

            ThirdBall = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(9.184, 10.078).mirror(),
                            new Pose(9.184, 15.078).mirror()))
                    .setConstantHeadingInterpolation(180 -150)
                    .build();

            DriveForward = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(9.184, 15.078).mirror(),
                            new Pose(9.184, 13.078).mirror()))
                    .setConstantHeadingInterpolation(180 - 270)
                    .build();

            ShootCorner = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(11.184, 10.078).mirror(),
                            new Pose(55.899, 20.788).mirror()))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(180 - 180),
                            Math.toRadians(180 -295))
                    .build();

            Leave = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(55.821, 16.757).mirror(),
                            new Pose(30, 15.902).mirror()))
                    .setLinearHeadingInterpolation(Math.toRadians(180 - 291), Math.toRadians(180))
                    .build();
        }
    }
}
