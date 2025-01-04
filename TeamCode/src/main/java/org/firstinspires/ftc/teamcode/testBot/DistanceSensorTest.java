package org.firstinspires.ftc.teamcode.testBot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class DistanceSensorTest extends OpMode {
    DistanceSensor distanceSensor;

    @Override
    public void init() {
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distSensor");
    }

    @Override
    public void loop() {
        telemetry.addData("Distance (in.)", distanceSensor.getDistance(DistanceUnit.INCH));
        telemetry.update();
    }
}
