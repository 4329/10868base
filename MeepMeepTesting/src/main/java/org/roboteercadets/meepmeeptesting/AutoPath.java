package org.roboteercadets.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;

public interface AutoPath {
    Pose2d getStartingPose();
    void setActionBuilder(TrajectoryActionBuilder builder);
    TrajectoryActionBuilder getActionBuilder();
    void createPaths();
}
