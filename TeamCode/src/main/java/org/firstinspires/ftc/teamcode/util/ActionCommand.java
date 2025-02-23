package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.CommandBase;

public class ActionCommand extends CommandBase {
    private Action action;
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private boolean stillRunning = true;
    public ActionCommand(Action action) {
        this.action = action;
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        TelemetryPacket packet = new TelemetryPacket();
        action.preview(packet.fieldOverlay());
        stillRunning = action.run(packet);
        dashboard.sendTelemetryPacket(packet);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return stillRunning;
    }
}
