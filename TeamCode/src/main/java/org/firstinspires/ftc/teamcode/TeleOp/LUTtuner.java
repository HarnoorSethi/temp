package org.firstinspires.ftc.teamcode.teleOp;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.util.ScoringGoal;
import org.firstinspires.ftc.teamcode.commands.FeedAndShoot;
import org.firstinspires.ftc.teamcode.subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

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
        shooter = new Shooter(hardwareMap,telemetry);
        intake = new Intake(hardwareMap);
        feedAndShoot = new FeedAndShoot(shooter, intake);
    }

    @Override
    public void loop() {
        new GamepadButton(driver, GamepadKeys.Button.DPAD_UP).whenPressed(new InstantCommand(() ->{
            feedAndShoot.distanceOffset += 50;
        }));
        new GamepadButton(driver, GamepadKeys.Button.DPAD_DOWN).whenPressed(new InstantCommand(() ->{
            feedAndShoot.distanceOffset -= 50;
        }));
        feedAndShoot.execute();

        telemetry.addData("Distance", drivebase.getPose().distanceFrom(scoringGoal.getPose()));

    }
}
