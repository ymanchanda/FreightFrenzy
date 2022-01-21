package org.firstinspires.ftc.teamcode.team10515.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.drivers.RevServo;
import org.firstinspires.ftc.teamcode.team10515.states.DropperRightStateMachine;

public class DropperRightSubsystem implements ISubsystem<DropperRightStateMachine, DropperRightStateMachine.State> {
    public static DropperRightStateMachine dropperRightStateMachine;
    private RevServo dropperRightServo;

    public DropperRightSubsystem(RevServo dropperRightServo){
        setDropperRightStateMachine(new DropperRightStateMachine());
        setDropperRightServo(dropperRightServo);
    }

    @Override
    public DropperRightStateMachine getStateMachine() {
        return dropperRightStateMachine;
    }

    @Override
    public DropperRightStateMachine.State getState() {
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
        return "Dropper Right Subsystem";
    }

    @Override
    public void writeToTelemetry(Telemetry telemetry) {

    }

    @Override
    public void update(double dt) {
        getStateMachine().update(dt);
        getDropperRightServo().setPosition(getState().getPosition());
    }

    public static void setDropperRightStateMachine(DropperRightStateMachine dropperRightStateMachine){
        DropperRightSubsystem.dropperRightStateMachine = dropperRightStateMachine;
    }

    public RevServo getDropperRightServo(){
        return dropperRightServo;
    }


    public void setDropperRightServo(RevServo dropperRightServo){
        this.dropperRightServo = dropperRightServo;
    }

}
