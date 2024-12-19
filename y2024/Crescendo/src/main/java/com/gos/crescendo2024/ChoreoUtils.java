package com.gos.crescendo2024;

import choreo.Choreo;
import choreo.trajectory.Trajectory;

public class ChoreoUtils {

    public static MaybeFlippedPose2d getPathStartingPose(String pathName) {
        Trajectory<?> trajectory = Choreo.loadTrajectory(pathName).orElseThrow();
        return new MaybeFlippedPose2d(trajectory.getInitialPose(false).get(), trajectory.getInitialPose(true).get());
    }
}
