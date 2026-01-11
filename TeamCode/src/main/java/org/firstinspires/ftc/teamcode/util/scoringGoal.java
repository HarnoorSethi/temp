package org.firstinspires.ftc.teamcode.Util;

import com.pedropathing.geometry.Pose;

/**
 * Defines fixed scoring goal positions and AprilTag IDs for both alliances.
 * Red goal is the mirror of the blue goal across the center line (x = 72, field width = 144 in).
 */
public enum ScoringGoal {

    BLUE(new Pose(3, 137), 20),
    RED(new Pose(144.0-3, 137), 24);



    private final Pose pose;
    private final int aprilTagId;

    ScoringGoal(Pose pose, int aprilTagId) {
        this.pose = pose;
        this.aprilTagId = aprilTagId;
    }

    public Pose getPose() {
        return pose;
    }

    public int getAprilTagId() {
        return aprilTagId;
    }
}