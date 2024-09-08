package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous //Tells the Driver Station to put this OpMode in the Autonomous section. In this example, the OpMode is called testBotWithMotors
public class testBotWithMotors extends LinearOpMode{ //For Autonomous programs, We extend the default LinearOpMode (which has a bunch of behind the scenes code) with what we want the robot to do
    //We can initialize some values here, before we press start on the driver station
    double motorSpeed = 0.5; //Motor speed ranges from -1 to 1. Note that because of motor orientation,
    // a positive speed could be forward or backward power, which is accounted for by setDirection().
    // You have to test it to find out.

    //This is what gets run when we press init on the driver station
    @Override
    public void runOpMode(){
        //Initialize motors here
        DcMotor fl = hardwareMap.dcMotor.get("frontLeft"); fl.setDirection(DcMotorSimple.Direction.REVERSE);
        DcMotor fr = hardwareMap.dcMotor.get("frontRight");
        DcMotor dl = hardwareMap.dcMotor.get("backLeft"); dl.setDirection(DcMotorSimple.Direction.REVERSE);
        DcMotor dr = hardwareMap.dcMotor.get("backRight");

        waitForStart(); //Wait for us to press start on the driver station

        //Run the motor for 2 seconds
        fl.setPower(motorSpeed);
        fr.setPower(motorSpeed);
        dl.setPower(motorSpeed);
        dr.setPower(motorSpeed);
        sleep(2000);
    }
}
