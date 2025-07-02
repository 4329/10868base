package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.actions.MecanumActions;
import org.firstinspires.ftc.teamcode.commands.ElevatorPositionCommand;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OTOSLocalizerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TelemetryUpdateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VoltageSubsystem;
import org.firstinspires.ftc.teamcode.util.ActionCommand;
import org.firstinspires.ftc.teamcode.util.ElevatorPosition;

@Autonomous(name = "Simple Red Auto")
public class SimpleRedAuto extends CommandOpMode {
    private MecanumDriveSubsystem driveSubsystem;
    private OTOSLocalizerSubsystem otosLocalizerSubsystem;
    private TelemetryUpdateSubsystem telemetryUpdateSubsystem;
    private VoltageSubsystem voltageSubsystem;
    private ElevatorSubsystem elevatorSubsystem;

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        driveSubsystem = new MecanumDriveSubsystem(hardwareMap);
        otosLocalizerSubsystem = new OTOSLocalizerSubsystem(hardwareMap);
        telemetryUpdateSubsystem = new TelemetryUpdateSubsystem(telemetry);
        voltageSubsystem = new VoltageSubsystem(hardwareMap);
        elevatorSubsystem = new ElevatorSubsystem(hardwareMap, telemetry);

        Pose2d startingPose = new Pose2d(24, -48, Math.toRadians(90));
        otosLocalizerSubsystem.setPose(new Pose2d(24, -48, Math.toRadians(90)));

        MecanumActions actions = new MecanumActions(driveSubsystem, otosLocalizerSubsystem, voltageSubsystem);
        TrajectoryActionBuilder trajBuilder = actions.actionBuilder(startingPose);
        Action firstSplineAction = trajBuilder.splineTo(new Vector2d(48, -24), Math.toRadians(0)).build();
        Action secondSplineAction = trajBuilder.splineTo(new Vector2d(24, -48), Math.toRadians(90)).build();

        Command auto = new SequentialCommandGroup(
            new ActionCommand(firstSplineAction),
            new ElevatorPositionCommand(elevatorSubsystem, ElevatorPosition.UPTHING),
            new WaitCommand(1500),
            new ElevatorPositionCommand(elevatorSubsystem, ElevatorPosition.DOWN)
               /* try adding if this works... , new ActionCommand(secondSplineAction) */
        );

        //driveSubsystem.setDefaultCommand();
        register(telemetryUpdateSubsystem, otosLocalizerSubsystem);
        schedule(auto);
    }
}