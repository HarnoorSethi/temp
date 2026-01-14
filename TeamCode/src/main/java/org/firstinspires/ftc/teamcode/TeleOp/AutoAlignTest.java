package org.firstinspires.ftc.teamcode.TeleOp;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.autoAlign;
import org.firstinspires.ftc.teamcode.subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.util.ScoringGoal;

@TeleOp
public class AutoAlignTest extends OpMode {

    Drivebase drivebase;
    autoAlign align;
    Limelight3A ll;

    ScoringGoal scoringGoal = ScoringGoal.BLUE;

    @Override
    public void init() {
        ll = hardwareMap.get(Limelight3A.class, "ll");
        ll.start();
        ll.setPollRateHz(100);
        ll.pipelineSwitch(0);

        drivebase = new Drivebase(hardwareMap);
        drivebase.initiaizeTeleOp();
        drivebase.startTeleOp();

        drivebase.setStartingPose(new Pose(72, 72, Math.toRadians(90)));

        align = new autoAlign(drivebase, scoringGoal, ll);
    }

    @Override
    public void loop() {
        drivebase.periodic();

        // Toggle auto-align
        if (gamepad1.aWasPressed()) {
            align.alignOn = !align.alignOn;
            drivebase.setMovementVectors(0, 0, 0);
        }

        // Auto-align OR manual control (never both)
        if (align.alignOn) {
            align.execute();
        } else {
            drivebase.setMovementVectors(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x
            );
        }

        telemetry.addData("AutoAlign", align.alignOn);
        telemetry.addData("LL Offset (deg)",
                align.llOffset == autoAlign.LL_INVALID
                        ? "INVALID"
                        : Math.toDegrees(align.llOffset));
        telemetry.update();
    }
}
