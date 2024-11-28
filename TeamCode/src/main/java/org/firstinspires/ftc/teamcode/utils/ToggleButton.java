package org.firstinspires.ftc.teamcode.utils;

public class ToggleButton {
    boolean buttonWasPressed = false;

    public boolean activated(boolean button){
        if(button && !buttonWasPressed){
            buttonWasPressed = true;
            return true;
        }
        buttonWasPressed = button;
        return false;
    }
}
