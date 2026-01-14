package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.ScoringGoal;
import org.firstinspires.ftc.teamcode.commands.FeedAndShoot;
import org.firstinspires.ftc.teamcode.subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.util.ScoringGoal;
@TeleOp
public class TeleOpBlue extends CommandOpMode {

    Shooter shooter;
    Intake intake;
    Drivebase drivebase;
    FeedAndShoot feedAndShoot;
    GamepadEx driver;



    @Override
    public void initialize(){
        driver = new GamepadEx(gamepad1);
        shooter = new Shooter(hardwareMap, telemetry);
        intake = new Intake(hardwareMap);
        drivebase = new Drivebase(hardwareMap);
        feedAndShoot = new FeedAndShoot(shooter, intake);
        drivebase.setStartingPose(new Pose(72,72,Math.toRadians(90)));
        drivebase.startTeleOp();
        register(shooter,intake,drivebase);
        shooter.setDefaultCommand(feedAndShoot);
        drivebase.setDefaultCommand(new RunCommand(()->{
            drivebase.setMovementVectors(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }));
        intake.setDefaultCommand(
                new RunCommand(() ->
                        intake.setIntakePower(
                                driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) -
                                        driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)
                        ), intake)
        );
        new GamepadButton(driver, GamepadKeys.Button.A).whenPressed(new InstantCommand(feedAndShoot::toggleFire));
    }
    @Override
    public void run(){
        super.run();
        feedAndShoot.updateFeedAndShootDistance(drivebase.getPose().distanceFrom(org.firstinspires.ftc.teamcode.util.ScoringGoal.BLUE.getPose()));
    }
}
