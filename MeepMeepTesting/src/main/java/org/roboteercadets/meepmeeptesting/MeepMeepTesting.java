package org.roboteercadets.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

/**
 * Use this class to hack on new paths and stuff that can be copied into the regular project and incorporated into auto modes.
 */
public class MeepMeepTesting {
    private static TrajectoryActionBuilder builder;
    private static Pose2d specimenPickup = new Pose2d(51, -60, Math.toRadians(270));
    private static Pose2d specimenDropoff = new Pose2d(8, -30, Math.toRadians(90));

    private static void plopOnBotSpecimin() {
        builder = builder.lineToY(-30).lineToY(-40);
    }

    private static void moveToFirstPush() {
        builder = builder.splineToLinearHeading(new Pose2d(41, -10, Math.toRadians(270)), Math.toRadians(90));
    }

    private static void pushBlocks() {
        builder = builder.splineToConstantHeading(new Vector2d(46, -60), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(50, -10), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(56, -60), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(59.5, -10), Math.toRadians(270))
                .strafeTo(new Vector2d(61, -10))
                .strafeTo(new Vector2d(61, -60))
                .strafeTo(new Vector2d(51, -60));
    }

    private static void hangSpecimin(int number) {
        Vector2d thisDropoff = new Vector2d(specimenDropoff.position.x - (number*4), specimenDropoff.position.y);
        builder = builder.splineToLinearHeading(new Pose2d(thisDropoff, specimenDropoff.heading), specimenDropoff.heading)
                         .splineToSplineHeading(specimenPickup,  new Rotation2d(specimenDropoff.heading.real, specimenDropoff.heading.imag));
    }

    public static void main(String[] args) {
        Pose2d startingPose = new Pose2d(8, -60, Math.toRadians(90));

        MeepMeep meepMeep = new MeepMeep(1600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(80, 80, Math.toRadians(180), Math.toRadians(180), 12)
                .build();


        builder = myBot.getDrive().actionBuilder(startingPose);
        plopOnBotSpecimin();
        moveToFirstPush();
        pushBlocks();
        for (int i = 0; i < 4; i++) {
           hangSpecimin(i);
        }



        myBot.runAction(builder.build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

}