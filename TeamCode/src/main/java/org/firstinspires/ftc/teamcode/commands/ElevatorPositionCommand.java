package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.util.ElevatorPosition;

public class ElevatorPositionCommand extends CommandBase {
    private ElevatorSubsystem subsystem;
    private ElevatorPosition position;

    public ElevatorPositionCommand(ElevatorSubsystem subsystem, ElevatorPosition position) {
        this.subsystem = subsystem;
        this.position = position;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        subsystem.goToPosition(position);
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return subsystem.uThereYet();
    }
}
