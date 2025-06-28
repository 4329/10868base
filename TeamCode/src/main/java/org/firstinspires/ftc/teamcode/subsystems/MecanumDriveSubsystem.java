package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class MecanumDriveSubsystem extends SubsystemBase {

    private MecanumDrive mecanumDrive;
    private Motor leftFrontDrive;
    private Motor rightFrontDrive;
    private Motor leftBackDrive;
    private Motor rightBackDrive;

    public MecanumDriveSubsystem(HardwareMap hardwareMap) {
        this.leftFrontDrive = new Motor(hardwareMap, "LeftFrontDrive");
        this.rightFrontDrive = new Motor(hardwareMap, "RightFrontDrive");
        this.leftBackDrive = new Motor(hardwareMap, "LeftBackDrive");
        this.rightBackDrive = new Motor(hardwareMap, "RightBackDrive");

        leftFrontDrive.motor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFrontDrive.motor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBackDrive.motor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBackDrive.motor.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFrontDrive.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        mecanumDrive = new MecanumDrive(leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive);
        mecanumDrive.setRightSideInverted(false);
    }

    public void stop() {
        mecanumDrive.stop();
    }

    public void drive(double strafe, double forward, double turn) {
        // what should we call here?
        mecanumDrive.driveRobotCentric(strafe, forward, turn);
    }

}
