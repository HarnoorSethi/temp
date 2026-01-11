package org.firstinspires.ftc.teamcode.TeleOp;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Util.ScoringGoal;
import org.firstinspires.ftc.teamcode.commands.autoAlign;
import org.firstinspires.ftc.teamcode.subsystems.Drivebase;

import kotlinx.coroutines.SchedulerTaskKt;
import kotlinx.coroutines.flow.internal.SafeCollector;

public class AutoAlignTest extends OpMode {
    Drivebase drivebase;
    ScoringGoal scoringGoal = ScoringGoal.BLUE;
    autoAlign align;

    @Override
    public void init() {
        drivebase = new Drivebase(hardwareMap);
        align = new autoAlign(drivebase, scoringGoal);
        drivebase.setStartingPose(new Pose(72, 72, Math.toRadians(90)));
    }

    @Override
    public void loop() {
        align.execute();
        telemetry.addData("Target Heading", align.targetHeading);
        telemetry.addData("Current Heading", align.currentHeading);
        telemetry.addData("Rotation Power", align.rotationPower);
        telemetry.update();
    }
}
