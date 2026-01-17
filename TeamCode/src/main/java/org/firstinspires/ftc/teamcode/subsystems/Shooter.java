package org.firstinspires.ftc.teamcode.subsystems;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.concurrent.TimeUnit;
@Configurable
public class Shooter extends SubsystemBase {
    private final MotorEx shooter1, shooter2;
    private final MotorGroup shooters;

    public double lastEncoderPos = 0;


    private double voltage = 12.5;

    private static final double MAX_RPM = 6214.1428571;
    private static final double kS = 0.25;

    private double lastVelocity = 0.0;

    private boolean ballJustShot = false;
    private final ElapsedTime time = new ElapsedTime();
    private int shotCount = 0;

    private double acceleration = 0;

    private double maxDecel = 0;





    public double desiredHoodPos;

    private double feedPower;
    private final DcMotor feeder;
    public final PIDController velocityPID;



    public double targetVelocity = 0.0;
    public double currentVelocity = 0.0;
    private ArrayList<Double> velocityHistory = new ArrayList<>(Arrays.asList(0.0,0.0,0.0,0.0,0.0));
    private VoltageSensor battery;
    private ElapsedTime pollTimer;

    private ArrayList<Double> accelHistory = new ArrayList<>(Arrays.asList(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
    public double smoothedAcceleration = 0.0;


    public static double kP = 0.0007;
    public static double kI = 0.0006;
    public static double kD = 0.0;


    Telemetry telemetry;
    public Shooter(HardwareMap hardwareMap, Telemetry telemetry) {
        pollTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        shooter1 = new MotorEx(hardwareMap, "flywheel1");
        shooter2 = new MotorEx(hardwareMap, "flywheel2");
        shooter1.setInverted(true);
        battery = hardwareMap.voltageSensor.iterator().next();
        feeder = hardwareMap.get(DcMotor.class, "feeder2");
        this.telemetry = telemetry;



        shooters = new MotorGroup(shooter1, shooter2);

        // TODO: tune
        velocityPID = new PIDController(kP,kI,kD);
        velocityPID.setTolerance(150);
        velocityPID.setIntegrationBounds(-0.3/0.002, 0.3/0.002);






    }

    public boolean isReadyToShoot(){
        return velocityPID.atSetPoint();
    }


    public void setTargetVelocity(double velocity) {
        this.targetVelocity = velocity;
    }

    public void feed(){
        feeder.setPower(-1);
    }

    public void stopFeeder(){
        feeder.setPower(0);
    }
    public void reverseFeed(){
        feeder.setPower(1);
    }

    public void setPConstant(double P){
        velocityPID.setP(P);
    }

    public void getVelocity(){
        double updateInterval = 0.01;
        if (pollTimer.milliseconds() >= (updateInterval * 1000)) {

            int currentPos = shooter1.getCurrentPosition();
            double dTicks = currentPos - lastEncoderPos;
            lastEncoderPos = currentPos;
            double timeDelta = pollTimer.milliseconds() / 1000.0;
            double ticksPerSecond = dTicks / timeDelta;
            double flywheelVelocity = 60 * (ticksPerSecond/28.0) * (19.0/19.0);
            velocityHistory.remove(0);
            velocityHistory.add(flywheelVelocity);

            double avg = 0;
            for (double v : velocityHistory) avg += v;
            currentVelocity = avg / velocityHistory.size();



            lastVelocity = currentVelocity;


            pollTimer.reset();

        }



    }

    public void resetMaxDecel() {
        maxDecel = 0;
    }

    @Override
    public void periodic() {
        if (time.time(TimeUnit.SECONDS) > 2){
            velocityPID.clearTotalError();
            time.reset();
        }

        velocityPID.setPID(kP,kI,kD);


        getVelocity();
        double output = 0.15;
        if (currentVelocity != 0){
            output = velocityPID.calculate(currentVelocity, targetVelocity);
        }
        double ff = (targetVelocity)/(MAX_RPM) + kS; //Arbitrary values here, i think its a litlle too high.
        double voltageComp = voltage / battery.getVoltage();

        double power = voltageComp*(output+ff);
        telemetry.addData("power", power).addData("ff", ff);
        power = Range.clip(power, 0.00001, 1);
        shooters.set(power);



        // Todo: Detect a shot (tune threshold!)

        if (smoothedAcceleration < -70 && !ballJustShot) {
            if (!accelHistory.contains(0.0)){

                ballJustShot = true;
                shotCount++;


            }

        }


        // Reset once velocity recovers
        if (smoothedAcceleration >= 300) {
            ballJustShot = false;
        }

        // Update for next loop

        // Optional: log values for tuning

    }

}