package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.util.ScoringGoal;
import org.firstinspires.ftc.teamcode.subsystems.Drivebase;
@Configurable
public class autoAlign extends CommandBase {
    Drivebase drivebase;
    ScoringGoal scoringGoal;
    PIDController rotateController;
    public static double ki = 0;
    public static double kp = 0.3;
    public static double kd = 0;
    public double currentHeading = 0;
    public double targetHeading = 0;
    public double rotationPower = 0;


    public autoAlign(Drivebase drivebase , ScoringGoal scoringGoal){
        this.drivebase = drivebase;
        this.scoringGoal = scoringGoal;
        this.rotateController = new PIDController(kp,ki,kd);
    }


    @Override
    public void execute(){
        currentHeading = drivebase.getPose().getHeading();
        targetHeading = Math.atan2(scoringGoal.getPose().getY() - drivebase.getPose().getY(), scoringGoal.getPose().getX() - drivebase.getPose().getX());
        rotationPower = rotateController.calculate(currentHeading,targetHeading);
        drivebase.setMovementVectors(0,0, rotationPower);
    }
    @Override
    public boolean isFinished(){
        return false;
    }
}
