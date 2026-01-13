package org.firstinspires.ftc.teamcode.TeleOp;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.ScoringGoal;
import org.firstinspires.ftc.teamcode.commands.FeedAndShoot;
import org.firstinspires.ftc.teamcode.subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@TeleOp(name= "feed and shoot test")
public class FeedAndShootTest extends OpMode {
    Drivebase drivebase;
    Intake intake;
    Shooter shooter;
    ScoringGoal scoringGoal = ScoringGoal.BLUE;
    FeedAndShoot feedAndShoot;


    @Override
    public void init() {
        drivebase = new Drivebase(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap, telemetry);
        feedAndShoot = new FeedAndShoot(shooter,intake);
        drivebase.setStartingPose(new Pose(72,72, Math.toRadians(90)));
    }

    @Override
    public void loop() {
        feedAndShoot.updateFeedAndShootDistance(Math.min(Math.max(drivebase.getPose().distanceFrom(scoringGoal.getPose()), 135), 10));
        feedAndShoot.execute();
        intake.setIntakePower(1);
        shooter.periodic();
        drivebase.periodic();
        telemetry.addData("Distance", drivebase.getPose().distanceFrom(scoringGoal.getPose()));
        telemetry.addData("Velocity", shooter.currentVelocity);
        telemetry.addData("Target Velocity", shooter.targetVelocity);
        telemetry.addData("pose", drivebase.getPose().toString());
        telemetry.addData("scoring pose", scoringGoal.getPose().toString());

        telemetry.update();
    }
}
