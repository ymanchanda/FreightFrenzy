package org.firstinspires.ftc.teamcode.team10515.states;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.util.Namable;
import org.firstinspires.ftc.teamcode.team10515.subsystems.LiftSubsystem;

import java.util.function.DoubleConsumer;

public class LiftStateMachine implements IState<LiftStateMachine.State> {
    private static DoubleConsumer runExtension;
    private LiftSubsystem liftSubsystem;
    private State state;
    private State desiredState;

    public LiftStateMachine(LiftSubsystem liftSubsystem) {
        setLiftSubsystem(liftSubsystem);
        setState(State.IDLE);
        setDesiredState(State.IDLE);
    }

    @Override
    public void updateState(State state) {
        setDesiredState(state);
    }

    @Override
    public boolean hasReachedStateGoal() {
        return getLiftSubsystem().closeToSetpoint(1 / 4d) && liftSubsystem.getExtensionProfile().isDone();
    }

    @Override
    public boolean hasReachedStateGoal(State state) {
        return state.equals(getState()) && hasReachedStateGoal();
    }

    @Override
    public boolean attemptingStateChange() {
        return !getState().equals(getDesiredState());
    }

    @Override
    public State getState() {
        return state;
    }

    @Override
    public State getDesiredState() {
        return desiredState;
    }

    @Override
    public String getName() {
        return "Lift State Machine";
    }

    @Override
    public void writeToTelemetry(Telemetry telemetry) {

    }

    @Override
    public void update(double dt) {
        if(attemptingStateChange()) {
            if(getDesiredState().equals(State.EXTEND)) {
                setState(getDesiredState());
                if(getRunExtension() != null) {
                    getRunExtension().accept(LiftSubsystem.getDesiredSetpoint());
                }
            } else if(!getDesiredState().equals(State.EXTEND)) {
                setState(getDesiredState());
                if(getRunExtension() != null) {
                    getRunExtension().accept(LiftSubsystem.getDesiredSetpoint());
                }
            }
        }
    }

    private void setState(State state) {
        this.state = state;
    }

    private void setDesiredState(State desiredState) {
        this.desiredState = desiredState;
    }

    public enum State implements Namable {
        IDLE("Idle"), EXTEND("Extend"), RETRACT("Retract");

        private final String name;

        State(final String name) {
            this.name = name;
        }

        @Override
        public String getName() {
            return name;
        }
    }

    public LiftSubsystem getLiftSubsystem() {
        return liftSubsystem;
    }

    public void setLiftSubsystem(LiftSubsystem liftSubsystem) {
        this.liftSubsystem = liftSubsystem;
    }

    public static DoubleConsumer getRunExtension() {
        return runExtension;
    }

    public static void setRunExtension(DoubleConsumer runExtension) {
        LiftStateMachine.runExtension = runExtension;
    }
}
