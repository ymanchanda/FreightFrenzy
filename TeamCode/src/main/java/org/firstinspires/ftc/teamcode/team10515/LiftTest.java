package org.firstinspires.ftc.teamcode.team10515;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Lift Test", group="Main")
public class LiftTest extends FreightFrenzyRobot {

    double power = 0;

    @Override
    public void init() {
        super.init();
        telemetry.addData("Init", "Hello Storm Trooper");
        updateTelemetry(telemetry);
    }

    @Override
    public void loop() {
        super.loop();
        if(getEnhancedGamepad1().isA()){
            power = 0.25;
        }
        if(getEnhancedGamepad1().isX()){
            power = -0.25;
        }
        if(getEnhancedGamepad1().isB()){
            power = 0;
        }
        getMotors()[5].setPower(power);
        telemetry.addLine("Encoder Ticks: " + getMotors()[5].getCurrentEncoderTicks());
    }
}
