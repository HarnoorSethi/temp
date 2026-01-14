package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.util.ScoringGoal;

@Configurable
public class autoAlign extends CommandBase {

    // ===== CONSTANTS =====
    public static final double LL_INVALID = 1e9;

    // ===== SUBSYSTEMS =====
    private final Drivebase drivebase;
    private final ScoringGoal scoringGoal;
    private final Limelight3A ll;

    // ===== PID =====
    private final PIDController pid;

    public static double kp = 0.5;
    public static double ki = 0.0;
    public static double kd = 0.015;

    public static double maxPower = 0.6;
    public static double minPower = 0.07;
    public static double deadbandDeg = 0;

    // ===== STATE =====
    public boolean alignOn = false;
    public double llOffset = LL_INVALID;
    public double angleOffset = 0;
    public double error;

    private final ElapsedTime llTimer = new ElapsedTime();

    public autoAlign(Drivebase drivebase, ScoringGoal scoringGoal, Limelight3A ll) {
        this.drivebase = drivebase;
        this.scoringGoal = scoringGoal;
        this.ll = ll;

        pid = new PIDController(kp, ki, kd);
        llTimer.reset();
    }

    @Override
    public void execute() {
        if (!alignOn) return;

        pid.setPID(kp, ki, kd);

        // ================= LIMELIGHT =================
        LLResult result = ll.getLatestResult();

        if (result != null && result.isValid()
                && !result.getFiducialResults().isEmpty()) {

            llOffset = Math.toRadians(
                    result.getFiducialResults().get(0).getTargetXDegrees()
            ) * 3;
            llTimer.reset();

        } else if (llTimer.seconds() > 0.1) {
            llOffset = LL_INVALID;
        }

        // ================= POSE HEADING =================
        double robotHeading = drivebase.getPose().getHeading();

        double goalAngle = Math.atan2(
                scoringGoal.getPose().getY() - drivebase.getPose().getY(),
                scoringGoal.getPose().getX() - drivebase.getPose().getX()
        );

        // Shooter is on the BACK
        double targetHeading = normalize(goalAngle + angleOffset);

        error = shortestAngleError(targetHeading, robotHeading);

        // Prefer Limelight when valid
        if (llOffset != LL_INVALID) {
            error = llOffset;
        }

        // ================= DEADZONE =================
        if (Math.abs(Math.toDegrees(error)) < deadbandDeg) {
            drivebase.setMovementVectors(0, 0, 0);
            pid.reset();
            return;
        }

        // ================= PID =================
        double output = pid.calculate(error);

        output = clamp(output, -maxPower, maxPower);

        // Static friction compensation
        if (Math.abs(output) < minPower) {
            output = Math.copySign(minPower, output);
        }

        drivebase.setMovementVectors(0, 0, output);
    }

    @Override
    public boolean isFinished() {
        if (Math.abs(Math.toDegrees(error)) < deadbandDeg) {
            drivebase.setMovementVectors(0, 0, 0);
            pid.reset();
            return true;
        }
        else return false;
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

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}
