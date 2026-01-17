package org.firstinspires.ftc.teamcode.TeleOp;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.util.ScoringGoal;

@TeleOp(name = "Angle Debug Test (NO MOTORS)", group = "TEST")
public class AngleDebugTest extends OpMode {

    private Drivebase drivebase;
    private Limelight3A ll;

    private final ScoringGoal scoringGoal = ScoringGoal.BLUE;

    // Debug values
    private double robotHeading;
    private double goalAngle;
    private double targetHeading;
    private double errorRad;
    private double errorDeg;

    private double llOffset = 1e9;

    @Override
    public void init() {

        drivebase = new Drivebase(hardwareMap);

        ll = hardwareMap.get(Limelight3A.class, "ll");
        ll.start();
        ll.setPollRateHz(100);
        ll.pipelineSwitch(0);

        // Set known pose so math is deterministic
        drivebase.setStartingPose(new Pose(72, 72, Math.toRadians(90)));
        drivebase.startTeleOp();
    }

    @Override
    public void loop() {
        drivebase.periodic();

        // ================= CURRENT HEADING =================
        robotHeading = drivebase.getPose().getHeading();

        // ================= GOAL ANGLE =================
        goalAngle = Math.atan2(
                scoringGoal.getPose().getY() - drivebase.getPose().getY(),
                scoringGoal.getPose().getX() - drivebase.getPose().getX()
        );

        targetHeading = normalize(goalAngle + Math.PI);

        // ================= ERROR =================
        errorRad = shortestAngleError(targetHeading, robotHeading);
        errorDeg = Math.toDegrees(errorRad);

        // ================= LIMELIGHT =================
        LLResult result = ll.getLatestResult();

        if (result != null
                && result.isValid()
                && !result.getFiducialResults().isEmpty()
                && result.getFiducialResults().get(0).getFiducialId() == scoringGoal.getAprilTagId()) {

            llOffset = -Math.toRadians(
                    result.getFiducialResults().get(0).getTargetXDegrees()
            );
        } else {
            llOffset = 1e9;
        }

        // ================= TELEMETRY =================
        telemetry.addLine("==== ANGLE DEBUG (NO MOTION) ====");
        telemetry.addData("Robot Heading (deg)", Math.toDegrees(robotHeading));
        telemetry.addData("Goal Heading  (deg)", Math.toDegrees(goalAngle));
        telemetry.addData("Target Heading(deg)", Math.toDegrees(targetHeading));
        telemetry.addData("Error (deg)", errorDeg);

        telemetry.addData(
                "LL Offset (deg)",
                llOffset == 1e9 ? "INVALID" : Math.toDegrees(llOffset)
        );

        telemetry.update();


    }

    // ================= HELPERS =================

    private double shortestAngleError(double target, double current) {
        double error = target - current;
        while (error > Math.PI) error -= 2 * Math.PI;
        while (error < -Math.PI) error += 2 * Math.PI;
        return error;
    }

    private double normalize(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}
