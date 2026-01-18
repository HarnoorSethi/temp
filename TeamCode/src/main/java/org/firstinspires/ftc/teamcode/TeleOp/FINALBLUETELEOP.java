package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.commands.FeedAndShoot;
import org.firstinspires.ftc.teamcode.commands.autoAlign;
import org.firstinspires.ftc.teamcode.subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.util.ScoringGoal;

import java.io.File;

@TeleOp(name = "FINAL BLUE TELEOP")
public class FINALBLUETELEOP extends CommandOpMode {

    // Subsystems
    private Drivebase drivebase;
    private Shooter shooter;
    private Intake intake;

    // Commands
    private FeedAndShoot feedAndShoot;
    private autoAlign autoAlign;

    // Input
    private GamepadEx driver;
    private GamepadEx operator;

    // Vision
    private Limelight3A ll;

    private final ScoringGoal scoringGoal = ScoringGoal.BLUE;

    @Override
    public void initialize() {



        // ================= HARDWARE =================
        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);

        drivebase = new Drivebase(hardwareMap);
        shooter = new Shooter(hardwareMap, telemetry);
        intake = new Intake(hardwareMap);

        ll = hardwareMap.get(Limelight3A.class, "ll");
        ll.start();
        ll.setPollRateHz(100);
        ll.pipelineSwitch(0);

        // ================= POSE =================
        drivebase.setStartingPose(readFinalPoseFromFile());
        drivebase.startTeleOp();

        // ================= COMMANDS =================
        feedAndShoot = new FeedAndShoot(shooter, intake);


        autoAlign = new autoAlign(drivebase, scoringGoal, ll);

        // ================= REGISTER SUBSYSTEMS =================
        register(drivebase, shooter, intake);



        // ================= DEFAULT COMMANDS =================

        // DRIVE (manual unless auto-align is on)
        drivebase.setDefaultCommand(
                new RunCommand(() -> {

                    if (autoAlign.alignOn){
                        drivebase.alignDrive(-gamepad1.left_stick_y,
                                -gamepad1.left_stick_x);
                    }else {
                        drivebase.setMovementVectors(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x,
                                -gamepad1.right_stick_x
                        );
                    }
                }, drivebase)
        );

        // SHOOTER / FEED
        shooter.setDefaultCommand(feedAndShoot);

        // INTAKE (manual unless firing)
        intake.setDefaultCommand(
                new RunCommand(() -> {
                    if (!feedAndShoot.fire) {
                        intake.setIntakePower(
                                driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)
                                        - driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)
                        );
                    }
                }, intake)
        );

        // ================= BUTTON BINDINGS =================
        new GamepadButton(operator, GamepadKeys.Button.X)
                .whenPressed(() -> {
                    autoAlign.alignOn = !autoAlign.alignOn;
                    autoAlign.snap = !autoAlign.snap;
                    feedAndShoot.distanceOffset = 94 - feedAndShoot.distance;
                });

        // Toggle firing
        new GamepadButton(operator, GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(() -> {
                    autoAlign.alignOn = !autoAlign.alignOn;
                    drivebase.setMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, 0);
                }));

        // Toggle auto-align
        new GamepadButton(operator, GamepadKeys.Button.A)
                .whenPressed(new InstantCommand(() -> {
                    feedAndShoot.toggleFire();
                }))
                .whenReleased(new InstantCommand(() -> {
                    feedAndShoot.toggleFire();
                    autoAlign.alignOn = false;

                }));

    }

    private Pose readFinalPoseFromFile() {
        try {
            File poseFile = AppUtil.getInstance().getSettingsFile("finalPose.txt");
            if (!poseFile.exists()) {
                telemetry.addLine("No saved pose found — using default.");
                telemetry.update();
                return new Pose(72, 72, Math.PI / 2); // fallback
            }

            String data = ReadWriteFile.readFile(poseFile).trim(); // e.g. "x=56.000, y=9.000, heading=180.000"

            // Parse values using regex or split
            String[] parts = data.split(",");
            double x = Double.parseDouble(parts[0].split("=")[1]);
            double y = Double.parseDouble(parts[1].split("=")[1]);
            double headingDeg = Double.parseDouble(parts[2].split("=")[1]);

            return new Pose(x, y, Math.toRadians(headingDeg));
        } catch (Exception e) {
            telemetry.addLine("Failed to read pose: " + e.getMessage());
            telemetry.update();
            return new Pose(72, 72, Math.PI / 2); // fallback
        }
    }

    @Override
    public void run() {
        super.run();

        // ================= AUTO-ALIGN EXECUTION =================
        if (autoAlign.alignOn) {
            autoAlign.execute();
        }

        // ================= DISTANCE UPDATE =================
        feedAndShoot.updateFeedAndShootDistance(
                Math.max(
                        Math.min(
                                drivebase.getPose().distanceFrom(scoringGoal.getPose())
                                        + feedAndShoot.distanceOffset,
                                130
                        ),
                        10
                )
        );
        if (gamepad2.dpadUpWasPressed())feedAndShoot.distanceOffset += 5;
        if(gamepad2.dpadDownWasPressed()) feedAndShoot.distanceOffset -= 5;
        if (gamepad2.dpadLeftWasPressed())autoAlign.angleOffset += 2.5;
        if(gamepad2.dpadRightWasPressed()) autoAlign.angleOffset -= 2.5;

        // ================= TELEMETRY =================
        telemetry.addData("error", autoAlign.error);
        telemetry.addData("AutoAlign", autoAlign.alignOn);
        telemetry.addData(
                "LL Offset (deg)",
                autoAlign.llOffset == autoAlign.LL_INVALID
                        ? "INVALID"
                        : Math.toDegrees(autoAlign.llOffset)
        );
        telemetry.addData("Distance", drivebase.getPose().distanceFrom(scoringGoal.getPose()));
        telemetry.addData("Shooter Velocity", shooter.currentVelocity);
        telemetry.addData("Shooter Target", shooter.targetVelocity);
        //telemetry.addData("InDeadzone", autoAlign.inDeadzone);
        telemetry.addData("Offset", autoAlign.llOffset);
        telemetry.update();
    }
}
