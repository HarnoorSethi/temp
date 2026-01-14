package org.firstinspires.ftc.teamcode.teleOp;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.ScoringGoal;
import org.firstinspires.ftc.teamcode.commands.FeedAndShoot;
import org.firstinspires.ftc.teamcode.subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
@TeleOp
public class LUTtuner extends OpMode {
    Drivebase drivebase;
    Shooter shooter;
    Intake intake;
    FeedAndShoot feedAndShoot;
    ScoringGoal scoringGoal = ScoringGoal.BLUE;
    GamepadEx driver;

    @Override
    public void init() {
        drivebase = new Drivebase(hardwareMap);
        drivebase.setStartingPose(new Pose(72,72,Math.PI/2));
        shooter = new Shooter(hardwareMap,telemetry);
        intake = new Intake(hardwareMap);
        feedAndShoot = new FeedAndShoot(shooter, intake);
    }

    @Override
    public void loop() {
        if (gamepad1.aWasPressed()){
            feedAndShoot.toggleFire();
        }else if (gamepad1.a){

        }else {
            intake.setIntakePower(gamepad1.right_trigger-gamepad1.left_trigger);
        }
        if (gamepad1.dpadUpWasPressed())feedAndShoot.distanceOffset += 50;
        if(gamepad1.dpadDownWasReleased()) feedAndShoot.distanceOffset -= 50;
        feedAndShoot.execute();
        shooter.periodic();
        drivebase.periodic();

        telemetry.addData("Distance", drivebase.getPose().distanceFrom(scoringGoal.getPose()));
        telemetry.addData("pose", drivebase.getPose().toString());
        telemetry.addData("Target RPM", feedAndShoot.distanceOffset);
        telemetry.addData("RPM", shooter.currentVelocity);
        telemetry.update();
    }
}
