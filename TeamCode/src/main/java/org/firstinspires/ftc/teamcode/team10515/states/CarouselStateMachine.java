package org.firstinspires.ftc.teamcode.team10515.states;

import org.firstinspires.ftc.teamcode.lib.util.Namable;

public class CarouselStateMachine extends SimpleState<CarouselStateMachine.State> {
    public CarouselStateMachine() {
        super(State.IDLE);
    }

    @Override
    public String getName() {
        return "Carousel State Machine";
    }

    public enum State implements Namable {
        IDLE("Idle", 0), BLUE("Blue", 0.6d), RED("Red", -0.6d);

        private final String name;
        private double speed;

        State(final String name, double speed){
            this.name = name;
            this.speed = speed;
        }

        @Override
        public String getName() {
            return name;
        }

        public double getSpeed() {
            return this.speed;
        }
    }
}
