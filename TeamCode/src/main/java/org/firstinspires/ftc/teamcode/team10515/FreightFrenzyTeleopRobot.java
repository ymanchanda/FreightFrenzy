package org.firstinspires.ftc.teamcode.team10515;

import org.firstinspires.ftc.teamcode.lib.drivers.RevMotor;
import org.firstinspires.ftc.teamcode.lib.util.TimeProfiler;
import org.firstinspires.ftc.teamcode.team10515.auto.FFBase;


import java.util.Arrays;

/**
 * Motor naming convention:
 *     Drivetrain
 *         Back Right Wheel  -> RR
 *         Back Left Wheel   -> RL
 *     Intake
 *         Left Flywheel  -> INL
 *         Right Flywheel -> INR
 *     Outtake
 *         Left Extension  -> LL
 *         Right Extension -> LR
 * Servo naming convention:
 *     Outtake
 *
 *     End game
 *         Extension Blocker -> EXT
 * Misc. sensors naming convention:

 */
public abstract class FreightFrenzyTeleopRobot extends Robot {
    private TimeProfiler matchRuntime;
    protected FFBase drive;

    @Override
    public void init() {
        super.init();
        setMatchRuntime(new TimeProfiler(false));
    }

    @Override
    public void init_loop() {
        super.init_loop();
    }

    @Override
    public void start() {
        super.start();
        drive.getExpansionHubs().start();
        drive.robot.getDrive().start();
        Arrays.stream(getMotors()).forEach(RevMotor::resetEncoder);
        getMatchRuntime().start();
    }

    @Override
    public void loop() {
        super.loop();
        drive.getExpansionHubs().update(getDt());
        drive.update();
        drive.robot.getIntakeMotorSubsystem().update(getDt());
        drive.robot.getCarouselSubsystem().update(getDt());
        drive.robot.getDropperLeftSubsystem().update(getDt());
        drive.robot.getDropperRightSubsystem().update(getDt());
        drive.robot.getElevSubsystem().update(getDt());
    }


    @Override
    public void stop() {
        super.stop();
        drive.getExpansionHubs().stop();
        //drive.robot.getDrive().stop();
        drive.robot.getIntakeMotorSubsystem().stop();
        drive.robot.getCarouselSubsystem().stop();
        drive.robot.getDropperLeftSubsystem().stop();
        drive.robot.getDropperRightSubsystem().stop();
        drive.robot.getElevSubsystem().stop();
    }

    public TimeProfiler getMatchRuntime() {
        return matchRuntime;
    }

    public void setMatchRuntime(TimeProfiler matchRuntime) {
        this.matchRuntime = matchRuntime;
    }

}
