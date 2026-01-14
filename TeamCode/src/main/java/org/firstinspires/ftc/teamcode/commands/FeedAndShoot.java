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
        InterpLUT rpmLUT = new InterpLUT();


        rpmLUT.add(0,     2000);   // clamp low
        rpmLUT.add(55,  2400);
        rpmLUT.add(64,  2500);
        rpmLUT.add(81,  2500);
        rpmLUT.add(89,  2550);
        rpmLUT.add(99.7,  2600);
        rpmLUT.add(110,  2750);
        rpmLUT.add(122.9, 2900);
        rpmLUT.add(132, 3050);// clamp high
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

        shooter.setTargetVelocity(rpmLUT.get(distance + distanceOffset));


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