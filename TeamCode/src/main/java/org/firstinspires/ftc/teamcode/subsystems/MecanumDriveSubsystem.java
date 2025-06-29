package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.Time;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class MecanumDriveSubsystem extends SubsystemBase {

    private MecanumDrive mecanumDrive;
    private MotorEx leftFrontDrive;
    private MotorEx rightFrontDrive;
    private MotorEx leftBackDrive;
    private MotorEx rightBackDrive;

    public MecanumDriveSubsystem(HardwareMap hardwareMap) {
        this.leftFrontDrive = new MotorEx(hardwareMap, "LeftFrontDrive");
        this.rightFrontDrive = new MotorEx(hardwareMap, "RightFrontDrive");
        this.leftBackDrive = new MotorEx(hardwareMap, "LeftBackDrive");
        this.rightBackDrive = new MotorEx(hardwareMap, "RightBackDrive");

        leftFrontDrive.motorEx.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFrontDrive.motorEx.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBackDrive.motorEx.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBackDrive.motorEx.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFrontDrive.motorEx.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.motorEx.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.motorEx.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.motorEx.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontDrive.motorEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.motorEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.motorEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.motorEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

    public void setDrivePowers(PoseVelocity2d powers) {
        MecanumKinematics.WheelVelocities<Time> wheelVels = new MecanumKinematics(1).inverse(
                PoseVelocity2dDual.constant(powers, 1));

        double maxPowerMag = 1;
        for (DualNum<Time> power : wheelVels.all()) {
            maxPowerMag = Math.max(maxPowerMag, power.value());
        }

        leftFrontDrive.motorEx.setPower(wheelVels.leftFront.get(0) / maxPowerMag);
        leftBackDrive.motorEx.setPower(wheelVels.leftBack.get(0) / maxPowerMag);
        rightBackDrive.motorEx.setPower(wheelVels.rightBack.get(0) / maxPowerMag);
        rightFrontDrive.motorEx.setPower(wheelVels.rightFront.get(0) / maxPowerMag);
    }

    public void setDrivePowersInU(double leftFrontPower, double leftBackPower, double rightBackPower, double rightFrontPower) {
        leftFrontDrive.motorEx.setPower(leftFrontPower);
        leftBackDrive.motorEx.setPower(leftBackPower);
        rightBackDrive.motorEx.setPower(rightBackPower);
        rightFrontDrive.motorEx.setPower(rightFrontPower);
    }

}
