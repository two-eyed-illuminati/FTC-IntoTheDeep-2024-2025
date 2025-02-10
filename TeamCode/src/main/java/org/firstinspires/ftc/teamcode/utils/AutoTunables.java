package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.config.Config;

@Config
public class AutoTunables {
    public static double WAIT_TIME = 0.5;
    public static double SPECIMEN_START_HEIGHT = 17.5;
    public static double SPECIMEN_END_HEIGHT = 10.0;
    public static double SPECIMEN_DOWN_SLIDE_SPEED_MULTI = 0.6;
    public static double SPECIMEN_FORWARD = 17.5;
    public static double SPECIMEN_FORWARD_SPEED = 0.8;
    public static double SAMPLE_SIDEWAYS = 35.5;
    public static double SAMPLE_FORWARD = 26.0; //sum of what the targetGroundDistance should be and the specimen forward distance\
    public static double SAMPLE_GRAB_HEIGHT = 9.2;
    public static double GRAB_FINGER_OPEN_POS = 0.6; //Extra open so we have more error room
    public static double BASKET_X = 14.0;
    public static double BASKET_Y = 39.0;
}
