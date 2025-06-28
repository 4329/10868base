package org.firstinspires.ftc.teamcode.commands.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Actions;
import com.acmerobotics.roadrunner.HolonomicController;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeTrajectory;

import org.firstinspires.ftc.teamcode.roadrunner.Drawing;
import org.firstinspires.ftc.teamcode.roadrunner.messages.DriveCommandMessage;
import org.firstinspires.ftc.teamcode.roadrunner.messages.MecanumCommandMessage;
import org.firstinspires.ftc.teamcode.roadrunner.messages.PoseMessage;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

import java.util.List;

public class FollowTrajectoryCommandAction implements Action {
    public final TimeTrajectory timeTrajectory;
    private double beginTs = -1;
    private MecanumDriveSubsystem driveSubsystem;

    private final double[] xPoints, yPoints;

    public FollowTrajectoryCommandAction(TimeTrajectory t, MecanumDriveSubsystem drive) {
        this.timeTrajectory = t;
        this.driveSubsystem = drive;

        List<Double> disps = com.acmerobotics.roadrunner.Math.range(
                0, t.path.length(),
                Math.max(2, (int) Math.ceil(t.path.length() / 2)));
        xPoints = new double[disps.size()];
        yPoints = new double[disps.size()];
        for (int i = 0; i < disps.size(); i++) {
            Pose2d p = t.path.get(disps.get(i), 1).value();
            xPoints[i] = p.position.x;
            yPoints[i] = p.position.y;
        }
    }

    @Override
    public boolean run(@NonNull TelemetryPacket p) {
        double t;
        if (beginTs < 0) {
            beginTs = Actions.now();
            t = 0;
        } else {
            t = Actions.now() - beginTs;
        }

        if (t >= timeTrajectory.duration) {
            driveSubsystem.stop();
            return false;
        }

        Pose2dDual<Time> txWorldTarget = timeTrajectory.get(t);
        targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

        PoseVelocity2d robotVelRobot = updatePoseEstimate();

        PoseVelocity2dDual<Time> command = new HolonomicController(
                PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
        )
                .compute(txWorldTarget, localizer.getPose(), robotVelRobot);
        driveCommandWriter.write(new DriveCommandMessage(command));

        MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
        double voltage = voltageSensor.getVoltage();

        final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS,
                PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
        double leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
        double leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
        double rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
        double rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;
        mecanumCommandWriter.write(new MecanumCommandMessage(
                voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
        ));

        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
        rightFront.setPower(rightFrontPower);

        p.put("x", localizer.getPose().position.x);
        p.put("y", localizer.getPose().position.y);
        p.put("heading (deg)", Math.toDegrees(localizer.getPose().heading.toDouble()));

        Pose2d error = txWorldTarget.value().minusExp(localizer.getPose());
        p.put("xError", error.position.x);
        p.put("yError", error.position.y);
        p.put("headingError (deg)", Math.toDegrees(error.heading.toDouble()));

        // only draw when active; only one drive action should be active at a time
        Canvas c = p.fieldOverlay();
        drawPoseHistory(c);

        c.setStroke("#4CAF50");
        Drawing.drawRobot(c, txWorldTarget.value());

        c.setStroke("#3F51B5");
        Drawing.drawRobot(c, localizer.getPose());

        c.setStroke("#4CAF50FF");
        c.setStrokeWidth(1);
        c.strokePolyline(xPoints, yPoints);

        return true;
    }

    @Override
    public void preview(Canvas c) {
        c.setStroke("#4CAF507A");
        c.setStrokeWidth(1);
        c.strokePolyline(xPoints, yPoints);
    }
    @Override
    public void preview(@NonNull Canvas fieldOverlay) {
        Action.super.preview(fieldOverlay);
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        return false;
    }


}
