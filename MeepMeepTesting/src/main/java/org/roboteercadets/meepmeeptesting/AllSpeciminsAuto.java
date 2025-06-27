package org.roboteercadets.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;

public class AllSpeciminsAuto implements AutoPath {
    private Pose2d startingPose = new Pose2d(8, -60, Math.toRadians(90));
    private TrajectoryActionBuilder builder;

    @Override
    public Pose2d getStartingPose() {
        return startingPose;
    }

    @Override
    public void setActionBuilder(TrajectoryActionBuilder actionBuilder) {
        this.builder = actionBuilder;
    }

    @Override
    public TrajectoryActionBuilder getActionBuilder() {
        return builder;
    }

    @Override
    public void createPaths() {
        plopOnBotSpecimin();
        moveToFirstPush();
        pushBlocks();
        for (int i = -1; i < 4; i++) {
            hangSpecimin(i);
        }
    }

    private Pose2d specimenPickup = new Pose2d(51, -60, Math.toRadians(270));
    private Pose2d specimenDropoff = new Pose2d(8, -30, Math.toRadians(90));

    private void plopOnBotSpecimin() {
        builder = builder.lineToY(-30).lineToY(-40);
    }

    private void moveToFirstPush() {
        builder = builder.splineToLinearHeading(new Pose2d(41, -10, Math.toRadians(270)), Math.toRadians(90));
    }

    private void pushBlocks() {
        builder = builder.splineToConstantHeading(new Vector2d(46, -60), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(50, -10), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(56, -60), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(59.5, -10), Math.toRadians(270))
                .strafeTo(new Vector2d(61, -10))
                .strafeTo(new Vector2d(61, -60))
                .strafeTo(new Vector2d(51, -60));
    }

    private void hangSpecimin(int number) {
        Vector2d thisDropoff = new Vector2d(specimenDropoff.position.x - (number*4), specimenDropoff.position.y);
        builder = builder.splineToLinearHeading(new Pose2d(thisDropoff, specimenDropoff.heading), specimenDropoff.heading)
                .splineToSplineHeading(specimenPickup,  new Rotation2d(specimenDropoff.heading.real, specimenDropoff.heading.imag));
    }
}
