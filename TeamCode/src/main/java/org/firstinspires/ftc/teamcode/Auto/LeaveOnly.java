package org.firstinspires.ftc.teamcode.Auto;

import android.annotation.SuppressLint;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.subsystems.Drivebase;

import java.io.File;

@Autonomous(name = "Leave Only BLUE (FSM + Save Pose)", group = "Autonomous")
public class LeaveOnly extends CommandOpMode {

    // ================= FSM =================
    private enum PathState {
        START,
        LEAVE,
        DONE
    }

    private PathState pathState;

    // ================= Subsystems =================
    private Drivebase drivebase;

    // ================= Poses =================
    private final Pose startPose = new Pose(56, 8, Math.toRadians(90));

    private boolean poseSaved = false;

    @Override
    public void initialize() {

        drivebase = new Drivebase(hardwareMap);
        register(drivebase);

        drivebase.setStartingPose(startPose);

        Paths.init(drivebase.follower);

        pathState = PathState.START;
    }

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

            drivebase.follower.update();
            updateFSM();

            telemetry.addData("State", pathState);
            telemetry.addData("X", drivebase.getPose().getX());
            telemetry.addData("Y", drivebase.getPose().getY());
            telemetry.addData("Heading", drivebase.getPose().getHeading());
            telemetry.update();

            reset();
        }
    }

    // ================= FSM LOGIC =================
    private void updateFSM() {
        switch (pathState) {

            case START:
                drivebase.follower.followPath(Paths.leave);
                pathState = PathState.LEAVE;
                break;

            case LEAVE:
                if (!drivebase.follower.isBusy()) {
                    pathState = PathState.DONE;
                }
                break;

            case DONE:
                if (!poseSaved) {
                    saveFinalPose(drivebase.getPose());
                    poseSaved = true;
                }
                break;
        }
    }

    // ================= POSE SAVE =================
    private void saveFinalPose(Pose finalPose) {
        try {
            File poseFile = AppUtil.getInstance()
                    .getSettingsFile("finalPose.txt");

            @SuppressLint("DefaultLocale")
            String data = String.format(
                    "x=%.3f, y=%.3f, heading=%.3f",
                    finalPose.getX(),
                    finalPose.getY(),
                    Math.toDegrees(finalPose.getHeading())
            );

            ReadWriteFile.writeFile(poseFile, data);

        } catch (Exception e) {
            telemetry.addData("Pose Save", "Failed");
        }
    }

    // ================= PATH BANK =================
    public static class Paths {
        public static PathChain leave;

        public static void init(Follower follower) {

            leave = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(56.000, 8.000),
                                    new Pose(34.702, 8.821)
                            )
                    )
                    .setLinearHeadingInterpolation(
                            Math.toRadians(90),
                            Math.toRadians(90)
                    )
                    .build();
        }
    }
}
