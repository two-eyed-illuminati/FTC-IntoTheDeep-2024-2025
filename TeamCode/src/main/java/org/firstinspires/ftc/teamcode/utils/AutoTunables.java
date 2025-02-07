package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.config.Config;

@Config
public class AutoTunables {
    public static double WAIT_TIME = 0.5;
    public static double SPECIMEN_HEIGHT = 19.0;
    public static double SPECIMEN_FORWARD = 15.0;
    public static double SAMPLE_FORWARD = 16.0; //sum of what the targetGroundDistance should be and the specimen forward distance
    public static double GRAB_FINGER_OPEN_POS = RobotConstants.FINGER_OPEN_POS+(RobotConstants.FINGER_OPEN_POS-RobotConstants.FINGER_CLOSE_POS)/2.0; //Extra open so we have more error room
    public static double BASKET_X = 12.0;
    public static double BASKET_Y = 35.0;
}
