package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous //Tells the Driver Station to put this OpMode in the Autonomous section. In this example, the OpMode is called testBotWithTelemetry
public class testBotWithTelemetry extends LinearOpMode{ //For Autonomous programs, We extend the default LinearOpMode (which has a bunch of behind the scenes code) with what we want the robot to do
    //We can initialize some values here, before we press start on the driver station
    int telemetryValue = 8;

    //This is what gets run when we press init on the driver station
    @Override
    public void runOpMode(){
        waitForStart(); //Wait for us to press start on the driver station
        telemetry.addData("hi", 8);
        telemetry.update();
    }
}
