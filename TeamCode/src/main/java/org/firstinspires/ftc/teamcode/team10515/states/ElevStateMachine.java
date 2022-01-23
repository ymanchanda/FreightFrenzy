package org.firstinspires.ftc.teamcode.team10515.states;

import org.firstinspires.ftc.teamcode.lib.util.Namable;

public class ElevStateMachine extends SimpleState<ElevStateMachine.State> {
    public ElevStateMachine() { super(State.IDLE); }

    @Override
    public String getName() {
        return "Elevator State Machine";
    }

    public enum State implements Namable {
        IDLE("Idle", 0d),
        EXTENDTOP("Extend Top", 0.85d),
        EXTENDMIDDLE("Extend Middle",0.85d),
        RETRACT("Retract", -0.75d);

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
