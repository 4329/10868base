package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

import java.util.function.Supplier;

public class DriveByControllerCommand extends CommandBase {
    private static double REGULAR_MOTION_DIVISOR = 3.0;
    private static double SLOW_MOTION_DIVISOR = 6.0;
    private MecanumDriveSubsystem mecanumDriveSubsystem;
    private Supplier<Double> forwardDrive;
    private Supplier<Double> strafeDrive;
    private Supplier<Double> turnDrive;
    private Supplier<Boolean> speedBoost;
    private Supplier<Boolean> slowMode;

    public DriveByControllerCommand(MecanumDriveSubsystem mecanumDriveSubsystem, Supplier<Double> forwardDrive, Supplier<Double> strafeDrive, Supplier<Double> turnDrive, Supplier<Boolean> speedBoost, Supplier<Boolean> slowMode) {
        this.mecanumDriveSubsystem = mecanumDriveSubsystem;
        this.forwardDrive = forwardDrive;
        this.strafeDrive = strafeDrive;
        this.turnDrive = turnDrive;
        this.speedBoost = speedBoost;
        this.slowMode = slowMode;
    }

    @Override
    public void execute() {
        if (speedBoost.get()) {
            mecanumDriveSubsystem.drive(
                    strafeDrive.get(),
                    forwardDrive.get(),
                    turnDrive.get());
        }
        else if(slowMode.get()) {
            mecanumDriveSubsystem.drive(
                    strafeDrive.get() / SLOW_MOTION_DIVISOR,
                    forwardDrive.get() / SLOW_MOTION_DIVISOR,
                    turnDrive.get() / SLOW_MOTION_DIVISOR
            );
        }
        else {
            mecanumDriveSubsystem.drive(
                    strafeDrive.get() / REGULAR_MOTION_DIVISOR,
                    forwardDrive.get() / REGULAR_MOTION_DIVISOR,
                    turnDrive.get() / REGULAR_MOTION_DIVISOR
            );
        }
    }
}
