package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.config.Config;

//Allow configuration variables to be tuned without pushing code
//with FTC Dashboard (https://acmerobotics.github.io/ftc-dashboard/features#configuration-variables)
@Config
public class RobotConstants {
    public static double SLIDE_PIVOT_GROUND_HEIGHT = 9.5;
    public static double MAX_GROUND_DISTANCE = 10+13; //Max ground length of slides
    public static double MAX_PRESET_GROUND_DISTANCE = 10+13/2.0; //have a lower limit for max slide length for presets
    public static double MIN_GROUND_HEIGHT = 5;
    public static double MIN_GRAB_HEIGHT = 9.2;
    public static double MAX_GRAB_HEIGHT = 12.6;
    public static double FINGER_CLOSE_POS = 0.55;
    public static double FINGER_OPEN_POS = 0.85;
    public static double HAND_START_POS = 0.4044;
    public static double HAND_PARALLEL_POS = 0.4994; //Pos where hand is parallel with slides
    public static double HAND_START_ANGLE = Math.PI*15.0/180.0; //Acute angle between hand start and slides
    public static double WRIST_START_POS = 0.5921;
    public static double WRIST_PERPEN_POS = 0.265; //Pos where wrist is perpendicular to slides
}
