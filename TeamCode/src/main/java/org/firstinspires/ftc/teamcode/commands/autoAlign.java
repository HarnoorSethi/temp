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


    public static double maxPower = 1;
    public static double minPower = 0.00;
    public static double deadbandDeg = 2;


    // ===== STATE =====
    public boolean alignOn = false;
    public double llOffset = LL_INVALID;
    public double angleOffset = 0;
    public double error = 0;
    public boolean snap = false;






    private final ElapsedTime llTimer = new ElapsedTime();

    public autoAlign(Drivebase drivebase, ScoringGoal scoringGoal, Limelight3A ll) {
        this.drivebase = drivebase;
        this.scoringGoal = scoringGoal;
        this.ll = ll;

        pid = new PIDController(kp, ki, kd);
        llTimer.reset();

        pid.setTolerance(Math.toRadians(3));
    }



    @Override
    public void execute() {
        if (!alignOn) return;

        if (!snap){

        pid.setSetPoint(0);

        pid.setPID(kp, ki, kd);


        // ================= LIMELIGHT =================
        LLResult result = ll.getLatestResult();

        if (result != null && result.isValid()
                && !result.getFiducialResults().isEmpty() && result.getFiducialResults().get(0).getFiducialId() == scoringGoal.getAprilTagId()) {

            llOffset = Math.toRadians(
                    result.getFiducialResults().get(0).getTargetXDegrees()
            ) * 1;
            llTimer.reset();

            pid.setPID(kp,ki,kd);

        } else if (llTimer.seconds() > 0.1) {
            llOffset = LL_INVALID;
            pid.setPID(1,0, 0.1);
        }

        // ================= POSE HEADING =================
        double robotHeading = drivebase.getPose().getHeading();

        double goalAngle = Math.atan2(
                scoringGoal.getPose().getY() - drivebase.getPose().getY(),
                scoringGoal.getPose().getX() - drivebase.getPose().getX()
        );

        // Shooter is on the BACK
        double targetHeading = normalize(goalAngle);

        if ((pid.atSetPoint()) || (error == 0)){
        error = shortestAngleError(targetHeading, robotHeading);

        // Prefer Limelight when valid
        if (llOffset != LL_INVALID) {
            error = llOffset;
        }
        }





        // ================= PID =================
        double output = pid.calculate(error + angleOffset);

        output = clamp(output, -maxPower, maxPower);



        drivebase.alignRotate = output;
        }else {

            pid.setSetPoint(scoringGoal == ScoringGoal.BLUE ? 3 * (Math.PI/4) : Math.PI/4);
            drivebase.alignRotate = pid.calculate(drivebase.getPose().getHeading());


        }


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