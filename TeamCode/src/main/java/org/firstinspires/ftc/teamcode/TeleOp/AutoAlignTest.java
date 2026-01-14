package org.firstinspires.ftc.teamcode.TeleOp;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.ScoringGoal;
import org.firstinspires.ftc.teamcode.commands.autoAlign;
import org.firstinspires.ftc.teamcode.subsystems.Drivebase;

@TeleOp
public class AutoAlignTest extends OpMode {
    Drivebase drivebase;
    ScoringGoal scoringGoal = ScoringGoal.BLUE;
    autoAlign align;

    @Override
    public void init() {
        drivebase = new Drivebase(hardwareMap);
        drivebase.initiaizeTeleOp();
        drivebase.startTeleOp();

        align = new autoAlign(drivebase, scoringGoal);
        drivebase.setStartingPose(new Pose(72, 72, Math.toRadians(90)));
    }

    @Override
    public void loop() {
        drivebase.periodic();
        if(gamepad1.aWasPressed()) {
            align.alignOn = !align.alignOn;
            if (!align.alignOn){
                drivebase.setMovementVectors(0,0,0);
            }
        }
        align.execute();
        telemetry.addData("Target Heading", align.targetHeading);
        telemetry.addData("Current Heading", align.currentHeading);
        telemetry.addData("Rotation Power", align.rotationPower);

        telemetry.addData("on", align.alignOn);
        telemetry.update();
    }
}
