package org.firstinspires.ftc.teamcode.team10515.states;

import org.firstinspires.ftc.teamcode.lib.util.Namable;

public class IntakeStateMachine extends SimpleState<IntakeStateMachine.State> {
    public IntakeStateMachine() { super(State.IDLE); }

    @Override
    public String getName() {
        return "Intake State Machine";
    }

    public enum State implements Namable {
        IDLE("Idle", 0d),
        INTAKE("Intake", -0.75d),
        OUTTAKE("Outtake", 0.6d);

        private final String name;
        private final double speed;

        State(final String name, final double speed) {
            this.name  = name;
            this.speed = speed;
        }

        @Override
        public String getName() {
            return name;
        }

        public double getSpeed() {
            return speed;
        }
    }
}
