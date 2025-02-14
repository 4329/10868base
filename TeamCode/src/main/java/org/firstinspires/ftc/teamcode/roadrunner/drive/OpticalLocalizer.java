package org.firstinspires.ftc.teamcode.roadrunner.drive;

import android.util.Log;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class OpticalLocalizer implements Localizer {
    private SparkFunOTOS otos;

    public OpticalLocalizer(HardwareMap hardwareMap) {
        otos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");

        configureOtos();
    }

    private void configureOtos() {
        Log.i("OTOS", "configuring OTOS");
        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.RADIANS);

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
        SparkFunOTOS.Pose2D otosOffset = new SparkFunOTOS.Pose2D(0, 0, 0);
        otos.setOffset(otosOffset);

        otos.setLinearScalar(DriveConstants.LINEAR_SCALAR);
        otos.setAngularScalar(DriveConstants.ANGULAR_SCALAR);

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

    @NonNull
    @Override
    public Pose2d getPoseEstimate() {
        SparkFunOTOS.Pose2D otosPose = otos.getPosition();
        return new Pose2d(otosPose.x, otosPose.y, otosPose.h);
    }

    @Override
    public void setPoseEstimate(@NonNull Pose2d pose2d) {
        otos.resetTracking();

        SparkFunOTOS.Pose2D newPosition = new SparkFunOTOS.Pose2D(pose2d.getX(), pose2d.getY(), pose2d.getHeading());
        otos.setPosition(newPosition);
    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        SparkFunOTOS.Pose2D velPose = otos.getVelocity();
        return new Pose2d(velPose.x, velPose.y, velPose.h);
    }

    @Override
    public void update() {
       Log.d("OTOS", String.format("Current Location: ", otos.getPosition().toString()));
    }
}
