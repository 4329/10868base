package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class VoltageSubsystem extends SubsystemBase {
    private VoltageSensor voltageSensor;

    public VoltageSubsystem(HardwareMap hwMap) {
        this.voltageSensor = hwMap.voltageSensor.iterator().next();
    }

    public double getVoltage() {
        return voltageSensor.getVoltage();
    }
}
