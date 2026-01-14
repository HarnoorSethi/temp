package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Limelight Debug")
public class LimelightDebug extends OpMode {

    Limelight3A ll;

    @Override
    public void init() {
        ll = hardwareMap.get(Limelight3A.class, "ll");
        ll.start();
        ll.setPollRateHz(100);
        ll.pipelineSwitch(0);

        telemetry.addLine("Limelight Debug Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        LLResult result = ll.getLatestResult();

        if (result == null) {
            telemetry.addLine("Result: NULL");
        } else {
            telemetry.addData("Valid", result.isValid());
            //telemetry.addData("Latency (ms)", result.getLatency());

            if (result.isValid()) {
                telemetry.addData("Pipeline", result.getPipelineIndex());
                telemetry.addData("Has Fiducials", !result.getFiducialResults().isEmpty());

                if (!result.getFiducialResults().isEmpty()) {
                    telemetry.addData(
                            "Target X (deg)",
                            result.getFiducialResults().get(0).getTargetXDegrees()
                    );
                    telemetry.addData(
                            "Target Y (deg)",
                            result.getFiducialResults().get(0).getTargetYDegrees()
                    );
                    telemetry.addData(
                            "Target Area",
                            result.getFiducialResults().get(0).getTargetArea()
                    );
                }
            }
        }

        telemetry.update();
    }
}
