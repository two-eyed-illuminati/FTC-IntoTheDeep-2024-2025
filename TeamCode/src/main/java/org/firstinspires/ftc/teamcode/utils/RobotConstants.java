package org.firstinspires.ftc.teamcode.utils;

public class RobotConstants {
    public static final double SLIDE_PIVOT_GROUND_HEIGHT = 13.0;
    public static final double MAX_GROUND_DISTANCE = 10+13; //Max ground length of slides
    public static final double MAX_PRESET_GROUND_DISTANCE = 10+13/2.0; //have a lower limit for max slide length for presets
    public static final double MIN_GROUND_HEIGHT = 5;
    public static final double FINGER_CLOSE_POS = 0.309-0.035;
    public static final double FINGER_OPEN_POS = 0.386-0.035;
    public static final double HAND_START_POS = 1.0;
    public static final double HAND_PARALLEL_POS = 0.4; //Pos where hand is parallel with slides
    public static final double HAND_START_ANGLE = Math.PI*0; //Positive acute angle between hand start and slides
    public static final double WRIST_START_POS = 0.1341;
    public static final double WRIST_PERPEN_POS = 0.32+WRIST_START_POS; //Pos where wrist is perpendicular to slides
}
