package org.firstinspires.ftc.teamcode.teleOp;

import com.arcrobotics.ftclib.util.InterpLUT;
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
    InterpLUT rpmLUT;



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
        feedAndShoot.updateFeedAndShootDistance(Math.max(Math.min(drivebase.getPose().distanceFrom(scoringGoal.getPose()) + feedAndShoot.distanceOffset, 130), 10));
        feedAndShoot.execute();
        if (gamepad1.aWasPressed()){
            feedAndShoot.toggleFire();
        }else if (gamepad1.a){

        }else {
            intake.setIntakePower(gamepad1.right_trigger-gamepad1.left_trigger);
        }
        if (gamepad1.dpadUpWasPressed())feedAndShoot.distanceOffset += 50;
        if(gamepad1.dpadDownWasReleased()) feedAndShoot.distanceOffset -= 50;

        shooter.periodic();
        drivebase.periodic();
        telemetry.addData("Distance", drivebase.getPose().distanceFrom(scoringGoal.getPose()));
        telemetry.addData("Velocity", shooter.currentVelocity);
        telemetry.addData("Target Velocity", shooter.targetVelocity);
        telemetry.addData("pose", drivebase.getPose().toString());
        telemetry.addData("scoring pose", scoringGoal.getPose().toString());
        telemetry.addData("Scoring Goal", scoringGoal.toString());

        telemetry.update();
    }
}
