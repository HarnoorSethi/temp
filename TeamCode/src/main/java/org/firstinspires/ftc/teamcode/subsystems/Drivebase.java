package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class Drivebase extends SubsystemBase {

    public final Follower follower;
    private boolean isRobotCentric = true;
    private double powerMultiplier = 1;
    private ElapsedTime angularVelTimer;
    private double lastHeading;
    double currentHeading = Math.PI/2;

    public double distanceOffset = 0;



    public boolean isInBrakeMode = false;
    public PathChain pathChain;
    public Drivebase(HardwareMap hardwareMap){
        follower = Constants.createFollower(hardwareMap);
    }

    @Override
    public void periodic(){

        follower.update();
        currentHeading = follower.getHeading();
    }

    public void setStartingPose(Pose pose) {
        follower.setStartingPose(pose);
    }
    public void initiaizeTeleOp(){
        follower.startTeleopDrive();

    }
    public void setPathChain(PathChain pathChain){
        this.pathChain = pathChain;
    }

    public PathChain getPathChain(){
        return this.pathChain;
    }

    public void followPath(PathChain pathChain){
        follower.followPath(pathChain);
    }
    public void followPath(PathChain pathChain, boolean holdEnd){
        follower.followPath(pathChain, holdEnd);
    }
    public void startTeleOp(){
        follower.startTeleopDrive();
    }

    public void followPath(){
        follower.followPath(this.pathChain);
    }
    public Pose getPose(){
        return follower.getPose();
    }
    public void setPowerMultiplier(double powerMultiplier1){
        powerMultiplier = powerMultiplier1;
    }
    public void setMovementVectors(double forward, double lateral, double heading){


        follower.setTeleOpDrive(forward,1.2* lateral, heading, true);


    }

    public void setRobotCentric(){
        isRobotCentric = true;
    }
    public void setFieldCentric(){
        isRobotCentric = false;
    }
    public void toggleCentric() {
        isRobotCentric = !isRobotCentric;
    }
    public Follower getFollower() {
        return follower;
    }
}