package org.firstinspires.ftc.teamcode.team10515.states;

import org.firstinspires.ftc.teamcode.lib.util.Namable;
import org.firstinspires.ftc.teamcode.lib.util.Time;
import org.firstinspires.ftc.teamcode.lib.util.TimeUnits;

public class DropperRightStateMachine extends TimedState<DropperRightStateMachine.State> {
    public DropperRightStateMachine(){
        super(State.INIT);
    }

    @Override
    protected Time getStateTransitionDuration() {
        return new Time(1d, TimeUnits.SECONDS);
    }

    @Override
    public String getName() {
        return "Dropper Right State Machine";
    }

    public enum State implements Namable{
        PICKUP("Pickup", 1.0d), INIT("Init", 0.65d), DROPOFF("Dropoff", 0.45d);

        private final String name;
        private final double position;

        State(final String name, final double position){
            this.name = name;
            this.position = position;
        }

        public double getPosition() {
            return position;
        }

        @Override
        public String getName(){
            return name;
        }
    }
}