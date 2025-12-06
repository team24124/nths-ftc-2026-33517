package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.Pose;

public class SharedPoseStorage {
    public enum Team {RED, BLUE}

    public static Pose currentPose = new Pose();
    public static boolean poseAvailable = false;

    public static Team currentTeam = Team.RED;
    public static boolean teamAvailable = false;
}