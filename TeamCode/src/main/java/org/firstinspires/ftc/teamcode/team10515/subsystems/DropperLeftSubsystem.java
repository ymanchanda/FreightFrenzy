package org.firstinspires.ftc.teamcode.team10515.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.drivers.RevServo;
import org.firstinspires.ftc.teamcode.team10515.states.DropperLeftStateMachine;

public class DropperLeftSubsystem implements ISubsystem<DropperLeftStateMachine, DropperLeftStateMachine.State> {
    public static DropperLeftStateMachine dropperLeftStateMachine;
    private RevServo dropperLeftServo;

    public DropperLeftSubsystem(RevServo dropperLeftServo){
        setDropperLeftStateMachine(new DropperLeftStateMachine());
        setDropperLeftServo(dropperLeftServo);
    }

    @Override
    public DropperLeftStateMachine getStateMachine() {
        return dropperLeftStateMachine;
    }

    @Override
    public DropperLeftStateMachine.State getState() {
        return getStateMachine().getState();
    }

    @Override
    public void start() {

    }

    @Override
    public void stop() {

    }

    @Override
    public String getName() {
        return "Dropper Left Subsystem";
    }

    @Override
    public void writeToTelemetry(Telemetry telemetry) {

    }

    @Override
    public void update(double dt) {
        getStateMachine().update(dt);
        getDropperLeftServo().setPosition(getState().getPosition());
    }

    public static void setDropperLeftStateMachine(DropperLeftStateMachine dropperLeftStateMachine){
        DropperLeftSubsystem.dropperLeftStateMachine = dropperLeftStateMachine;
    }

    public RevServo getDropperLeftServo(){
        return dropperLeftServo;
    }


    public void setDropperLeftServo(RevServo dropperLeftServo){
        this.dropperLeftServo = dropperLeftServo;
    }

}
