package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TelemetryUpdateSubsystem;

@TeleOp(name = "TeleOp", group = "1")
public class Teleop extends CommandOpMode {
    private GamepadEx driver, operator;
    private MecanumDriveSubsystem mecanumDriveSubsystem ;
    private TelemetryUpdateSubsystem telemetryUpdateSubsystem;
   // private OTOSLocalizerSubsystem otosLocalizerSubsystem;
    private Command totalZeroCommandGroup;
    private Command driveMecanumCommand;

    @Override
    public void initialize() {
        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);
        telemetryUpdateSubsystem = new TelemetryUpdateSubsystem(telemetry);


        register(telemetryUpdateSubsystem);
    }
}
