package org.firstinspires.ftc.teamcode.team10515.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.drivers.RevMotor;
import org.firstinspires.ftc.teamcode.team10515.states.ElevStateMachine;

public class ElevSubsystem implements ISubsystem<ElevStateMachine, ElevStateMachine.State> {
    private static ElevStateMachine elevStateMachine;
    private RevMotor elevWheels;

    public ElevSubsystem(RevMotor elevMotor){
       setElevStateMachine(new ElevStateMachine());
       setElevWheels(elevMotor);
    }

    @Override
    public ElevStateMachine getStateMachine() {
        return elevStateMachine;
    }

    @Override
    public ElevStateMachine.State getState() {
        return getStateMachine().getState();
    }

    @Override
    public void start() {

    }

    @Override
    public void stop() {
        getElevWheels().setPower(0d);
    }

    @Override
    public void writeToTelemetry(Telemetry telemetry) {

    }

    @Override
    public void update(double dt) {
        getStateMachine().update(dt);
        if (getStateMachine().getState().equals(ElevStateMachine.State.EXTEND) && getElevWheels().getCurrentEncoderTicks() <= 2730)
            getElevWheels().setPower(getState().getSpeed());
        else if (getStateMachine().getState().equals(ElevStateMachine.State.RETRACT) && getElevWheels().getCurrentEncoderTicks() >= 50)
            getElevWheels().setPower(getState().getSpeed());
        else
            getElevWheels().setPower(0);
    }

    @Override
    public String getName() {
        return "Elev Subsystem";
    }

    private static void setElevStateMachine(ElevStateMachine elevStateMachine) {
        ElevSubsystem.elevStateMachine = elevStateMachine;
    }

    private void setElevWheels(RevMotor elevMotor){
        this.elevWheels = elevMotor;
    }
    private RevMotor getElevWheels(){
        return elevWheels;
    }
}
