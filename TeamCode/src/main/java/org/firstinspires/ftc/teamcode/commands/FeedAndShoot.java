package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.arcrobotics.ftclib.util.LUT;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.TeleOp.distanceLUT;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import org.firstinspires.ftc.teamcode.util.ScoringGoal;

import java.util.concurrent.TimeUnit;

public class FeedAndShoot extends CommandBase {



    private ScoringGoal goal;
    InterpLUT rpmLUT  ;



    private Shooter shooter;

    public boolean fire = false;

    public double distance = 200;
    public boolean rapidFire = false;
    public double distanceOffset = 0;








    public double lastP = 0.0024;
    Intake intake;
    public FeedAndShoot(Shooter shooter, Intake intake) {
        this.rpmLUT = new InterpLUT();


        rpmLUT.add(0,     2000);   // clamp low
        rpmLUT.add(61.5,  2200);
        rpmLUT.add(72.4,  2350);
        rpmLUT.add(81.5,  2450);
        rpmLUT.add(92.2,  2450);
        rpmLUT.add(100.6,  2600);
        rpmLUT.add(102.4,  2700);
        rpmLUT.add(110.6, 2700);
        rpmLUT.add(121.9, 2950);
        rpmLUT.add(122.5, 3000);
        rpmLUT.add(133.4, 3100);
        rpmLUT.add(140.8, 3250);// clamp high
        rpmLUT.createLUT();

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

        shooter.setTargetVelocity(rpmLUT.get(distance));


        if (fire && !rapidFire) {

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