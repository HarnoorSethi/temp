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

    public double distance = 190;
    public boolean rapidFire = false;
    public double distanceOffset = 0;








    public double lastP = 0.0024;
    Intake intake;
    public FeedAndShoot(Shooter shooter, Intake intake) {
        this.rpmLUT = new InterpLUT();


        rpmLUT.add(0,     2333);   // clamp low
        rpmLUT.add(41,  2333);
        rpmLUT.add(65,  2468);
        rpmLUT.add(85,  2739);
        rpmLUT.add(106,  3086);
        rpmLUT.add(140,  3500);
        rpmLUT.add(200,  4000);
        rpmLUT.createLUT();

        this.intake = intake;

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

        //shooter.setTargetVelocity(distanceOffset);
        shooter.setTargetVelocity(rpmLUT.get(distance));


        if (fire && shooter.isReadyToShoot()) {
            shooter.feed();
            intake.setIntakePower(1);


        }else{
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
