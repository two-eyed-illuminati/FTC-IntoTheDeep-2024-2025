package org.firstinspires.ftc.teamcode.utils;

//Allows for the creation of a button that only activates once per press, even if you hold it down
//Useful for things like toggling a servo
//Must be called on every iteration of the TeleOp loop for intended behavior
//Example Usage:
//ToggleButton toggleButton = new ToggleButton();
//public void loop(){
// if(toggleButton.activated(gamepad1.a)){
//     //do something
// }
//}
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
