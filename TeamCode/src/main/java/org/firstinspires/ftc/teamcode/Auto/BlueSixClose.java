
package org.firstinspires.ftc.teamcode.Auto;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.teamcode.commands.FeedAndShoot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.util.ScoringGoal;
@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable // Panels
public class BlueSixClose extends CommandOpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Drivebase db; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private Shooter shooter;
    private Intake intake;
    private final ScoringGoal scoringGoal = ScoringGoal.BLUE;
    private FeedAndShoot feedAndShoot;

    @Override
    public void initialize() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        shooter = new Shooter(hardwareMap,telemetry);
        intake = new Intake(hardwareMap);
        feedAndShoot = new FeedAndShoot(shooter,intake);

        db = new Drivebase(hardwareMap);
        db.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        paths = new Paths(db.follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void run() {
        db.follower.update(); // Update Pedro Pathing
        autonomousPathUpdate();
        feedAndShoot.updateFeedAndShootDistance(db.getPose().distanceFrom(scoringGoal.getPose()));

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", db.getPose().getX());
        panelsTelemetry.debug("Y", db.getPose().getY());
        panelsTelemetry.debug("Heading", db.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }


    public static class Paths {
        public PathChain ShootPreload;
        public PathChain GotToBalls;
        public PathChain IntakeBalls;
        public PathChain ShootFirstSpikeMark;
        public PathChain Leave;

        public Paths(Follower follower) {
            ShootPreload = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(23.453, 120.589),

                                    new Pose(62.434, 83.685)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))

                    .build();

            GotToBalls = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(62.434, 83.685),

                                    new Pose(38.903, 84.025)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))

                    .build();

            IntakeBalls = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(38.903, 84.025),

                                    new Pose(15.556, 83.778)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            ShootFirstSpikeMark = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(15.556, 83.778),

                                    new Pose(62.159, 83.833)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))

                    .build();

            Leave = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(62.159, 83.833),

                                    new Pose(25.234, 73.545)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(90))

                    .build();
        }
    }


    public void autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                // Shoot preload
                db.followPath(paths.ShootPreload);
                pathState++;
                break;

            case 1:
                if (!db.follower.isBusy()) {
                    feedAndShoot.toggleFire();

                    db.followPath(paths.GotToBalls);
                    pathState++;
                }
                break;

            case 2:
                if (!db.follower.isBusy()) {
                    db.followPath(paths.IntakeBalls);
                    pathState++;
                }
                break;

            case 3:
                if (!db.follower.isBusy()) {
                    db.followPath(paths.ShootFirstSpikeMark);
                    pathState++;
                }
                break;

            case 4:
                if (!db.follower.isBusy()) {
                    db.followPath(paths.Leave);
                    pathState++;
                }
                break;

            case 5:
                // Autonomous finished
                // You can stop updating paths here or add parking logic
                break;
        }
    }

}
    