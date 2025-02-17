package org.firstinspires.ftc.teamcode.roadrunner;

import android.util.Log;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class SparkFunOpticalLocalizer implements Localizer {
    private SparkFunOTOS otos;
    private DistanceUnit distanceUnit = DistanceUnit.INCH;
    private double linearScalar = 1.0;
    private AngleUnit angleUnit = AngleUnit.DEGREES;
    private double angularScalar = 1.0;

    private SparkFunOTOS.Pose2D currentPose = new SparkFunOTOS.Pose2D(0, 0, 0);
    private SparkFunOTOS.Pose2D currentVelocity = new SparkFunOTOS.Pose2D(0, 0, 0);
    private SparkFunOTOS.Pose2D currentAccel = new SparkFunOTOS.Pose2D(0, 0, 0);


    public SparkFunOpticalLocalizer(HardwareMap hardwareMap) {
        this.otos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");

        configureOtos();
    }

    public SparkFunOpticalLocalizer(HardwareMap hardwareMap, DistanceUnit distanceUnit, double linearScalar, AngleUnit angleUnit, double angularScalar) {
        this.otos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        this.distanceUnit = distanceUnit;
        this.linearScalar = linearScalar;
        this.angleUnit = angleUnit;
        this.angularScalar = angularScalar;

        configureOtos();
    }

    private void configureOtos() {
        Log.i("OTOS", "configuring OTOS");

        otos.setLinearUnit(distanceUnit);
        otos.setAngularUnit(angleUnit);

        // Assuming you've mounted your sensor to a robot and it's not centered,
        // you can specify the offset for the sensor relative to the center of the
        // robot. The units default to inches and degrees, but if you want to use
        // different units, specify them before setting the offset! Note that as of
        // firmware version 1.0, these values will be lost after a power cycle, so
        // you will need to set them each time you power up the sensor. For example, if
        // the sensor is mounted 5 inches to the left (negative X) and 10 inches
        // forward (positive Y) of the center of the robot, and mounted 90 degrees
        // clockwise (negative rotation) from the robot's orientation, the offset
        // would be {-5, 10, -90}. These can be any value, even the angle can be
        // tweaked slightly to compensate for imperfect mounting (eg. 1.3 degrees).
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
        otos.setOffset(offset);

        // Here we can set the linear and angular scalars, which can compensate for
        // scaling issues with the sensor measurements. Note that as of firmware
        // version 1.0, these values will be lost after a power cycle, so you will
        // need to set them each time you power up the sensor. They can be any value
        // from 0.872 to 1.127 in increments of 0.001 (0.1%). It is recommended to
        // first set both scalars to 1.0, then calibrate the angular scalar, then
        // the linear scalar. To calibrate the angular scalar, spin the robot by
        // multiple rotations (eg. 10) to get a precise error, then set the scalar
        // to the inverse of the error. Remember that the angle wraps from -180 to
        // 180 degrees, so for example, if after 10 rotations counterclockwise
        // (positive rotation), the sensor reports -15 degrees, the required scalar
        // would be 3600/3585 = 1.004. To calibrate the linear scalar, move the
        // robot a known distance and measure the error; do this multiple times at
        // multiple speeds to get an average, then set the linear scalar to the
        // inverse of the error. For example, if you move the robot 100 inches and
        // the sensor reports 103 inches, set the linear scalar to 100/103 = 0.971

        otos.setLinearScalar(linearScalar);
        otos.setAngularScalar(angularScalar);

        Log.i("OTOS", "calibrating IMU");
        otos.calibrateImu();
        otos.resetTracking();

        SparkFunOTOS.Pose2D initialPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        otos.setPosition(initialPosition);

        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        otos.getVersionInfo(hwVersion, fwVersion);

        Log.i("OTOS", "OTOS configured!");
        Log.i("OTOS", String.format("OTOS Hardware Version: v%d.%d", hwVersion.major, hwVersion.minor));
        Log.i("OTOS", String.format("OTOS Firmware Version: v%d.%d", fwVersion.major, fwVersion.minor));
    }


    @Override
    public void setPose(Pose2d pose) {
        currentPose = new SparkFunOTOS.Pose2D(pose.position.x, pose.position.y, pose.heading.toDouble());
        otos.resetTracking();
        otos.setPosition(currentPose);
    }

    @Override
    public Pose2d getPose() {
        return new Pose2d(currentPose.x, currentPose.y, currentPose.h);
    }

    @Override
    public PoseVelocity2d update() {
        otos.getPosVelAcc(currentPose, currentVelocity, currentAccel);
        return new PoseVelocity2d(new Vector2d(currentVelocity.x, currentVelocity.y), currentVelocity.h);
    }
}
