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

    public static double kp = 1;
    public static double ki = 0.0;
    public static double kd = 0.015;

    public static double maxPower = 1;
    public static double minPower = 0.00;
    public static double deadbandDeg = 2;

    // ===== STATE (ALL PRESERVED) =====
    public boolean alignOn = false;
    public double llOffset = LL_INVALID;
    public double angleOffset = 0;
    public double error = 0;

    public double theoreticalDBError = 0;
    public double dbAngleCalculated = 0;

    public boolean snap = false;
    public double targetHeading = 0;

    private final ElapsedTime llTimer = new ElapsedTime();

    public autoAlign(Drivebase drivebase, ScoringGoal scoringGoal, Limelight3A ll) {
        this.drivebase = drivebase;
        this.scoringGoal = scoringGoal;
        this.ll = ll;

        pid = new PIDController(kp, ki, kd);
        pid.setTolerance(Math.toRadians(3));

        llTimer.reset();
    }

    @Override
    public void execute() {

        // ================= LIMELIGHT =================
        LLResult result = ll.getLatestResult();

        if (result != null
                && result.isValid()
                && !result.getFiducialResults().isEmpty()
                && result.getFiducialResults().get(0).getFiducialId() == scoringGoal.getAprilTagId()) {

            llOffset = Math.toRadians(
                    -result.getFiducialResults().get(0).getTargetXDegrees()
            );

            llTimer.reset();
            pid.setPID(kp, ki, kd);

        } else if (llTimer.seconds() > 0.1) {
            llOffset = LL_INVALID;
        }

        if (!alignOn) {
            drivebase.alignRotate = 0;
            return;
        }

        // ================= SNAP MODE (UNCHANGED) =================
        if (snap) {
            pid.setSetPoint(
                    scoringGoal == ScoringGoal.BLUE
                            ? 3 * (Math.PI / 4) + Math.PI
                            : Math.PI + Math.PI / 4
            );

            drivebase.alignRotate = pid.calculate(drivebase.getPose().getHeading());
            return;
        }

        // ================= NON-SNAP (PID FIXED) =================

        pid.setPID(kp, ki, kd);

        double robotHeading = drivebase.getPose().getHeading();

        double goalAngle = Math.atan2(
                scoringGoal.getPose().getY() - drivebase.getPose().getY(),
                scoringGoal.getPose().getX() - drivebase.getPose().getX()
        );

        // Preserve original behavior: update targetHeading only when stable
        if (pid.atSetPoint() || targetHeading == 0) {
            targetHeading = normalize(goalAngle + Math.PI);
            dbAngleCalculated = Math.toDegrees(targetHeading);
        }

        // ================= ERROR CALCULATION =================
        error = shortestAngleError(targetHeading, robotHeading);
        theoreticalDBError = Math.toDegrees(error);

        // Prefer Limelight when valid (preserved behavior)
        if (llOffset != LL_INVALID) {
            error = llOffset;
        }

        // ================= PID (CORRECTED) =================
        // PID now treats ERROR as error, not measurement
        pid.setSetPoint(0);
        double output = pid.calculate(-error + angleOffset);

        output = clamp(output, -maxPower, maxPower);

        drivebase.alignRotate = output;
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
