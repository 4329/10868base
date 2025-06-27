package org.roboteercadets.meepmeeptesting;

import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

/**
 * Use this class to hack on new paths and stuff that can be copied into the regular project and incorporated into auto modes.
 */
public class MeepMeepTesting {
    private void runAuto() {
       MeepMeep meepMeep = new MeepMeep(1600);
       RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(80, 80, Math.toRadians(180), Math.toRadians(180), 12)
                .build();

       // change to whatever other auto you want...
       AutoPath auto = new AllSpeciminsAuto();

       TrajectoryActionBuilder builder = myBot.getDrive().actionBuilder(auto.getStartingPose());
       auto.setActionBuilder(builder);
       auto.createPaths();
       myBot.runAction(auto.getActionBuilder().build());

       meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
               .setBackgroundAlpha(0.95f)
               .addEntity(myBot)
               .start();
    }

    public static void main(String[] args) {
        new MeepMeepTesting().runAuto();
    }

}