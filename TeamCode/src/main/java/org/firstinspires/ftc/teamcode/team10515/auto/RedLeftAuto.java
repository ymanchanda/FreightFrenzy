package org.firstinspires.ftc.teamcode.team10515.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "RedLeft", group = "Concept")
public class RedLeftAuto extends LinearOpMode {
    static final double COUNTS_PER_INCH = (383.6 * 0.5)/(3.77953 * 3.1415);
    public void runOpMode(){
        waitForStart();
        FFAutoMethods autoMethods = new FFAutoMethods();
        autoMethods.init();
        autoMethods.forward(0.3, 20 * (int)COUNTS_PER_INCH);
        autoMethods.stop();
        autoMethods.turnLeft(0.3,90.0);
    }

}
