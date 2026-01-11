package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.LUT;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Util.distanceLUT;
import org.firstinspires.ftc.teamcode.util.ScoringGoal;

import java.util.concurrent.TimeUnit;

public class FeedAndShoot extends CommandBase {



    private ScoringGoal goal;


    private Shooter shooter;

    public boolean fire = false;

    public double distance = 200;
    public boolean rapidFire = false;
    public double distanceOffset = 0;








    public double lastP = 0.0024;
    Intake intake;
    public FeedAndShoot(Shooter shooter, Intake intake) {
 
        this.intake = intake;

        this.shooter = shooter;
        addRequirements(shooter);


    }
    public void updateFeedAndShootDistance(double distance){
        this.distance = distance;
    }

    public void toggleFire(){
        fire = !fire;
        if (!fire) {
            rapidFire = false;
        }

    }


    @Override
    public void execute() {
        double llTargetHeadingOffset = 0;


        //shooter.setTargetVelocity(distanceLUT.getRPM(distance));
        shooter.setTargetVelocity(distanceOffset);

        if (fire && shooter.isReadyToShoot() && !rapidFire) {
            rapidFire = true;
        }
        if (rapidFire){

            shooter.feed();
            intake.setIntakePower(1);

        }else {
            shooter.reverseFeed();


        }
        if (shooter.velocityPID.getPositionError() > 160){
            shooter.velocityPID.setPID(0.0024 * 3, 0.002 * 2, 0);

        }else {

            shooter.velocityPID.setPID(0.003 , 0.0015 , 0);
        }

        if (lastP != shooter.velocityPID.getP()){
            shooter.velocityPID.clearTotalError();
        }

    }
/*
    @Override
    public boolean isFinished(){
        return (shooter.getShotsFired() == 3);
    }

 */


}