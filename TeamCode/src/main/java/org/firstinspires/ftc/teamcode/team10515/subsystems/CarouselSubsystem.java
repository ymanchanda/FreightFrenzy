package org.firstinspires.ftc.teamcode.team10515.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.drivers.RevMotor;
import org.firstinspires.ftc.teamcode.team10515.states.CarouselStateMachine;

public class CarouselSubsystem implements ISubsystem<CarouselStateMachine, CarouselStateMachine.State> {
    private static CarouselStateMachine carouselStateMachine;
    private RevMotor carousel;

    public CarouselSubsystem(RevMotor carousel){
        setCarouselStateMachine(new CarouselStateMachine());
        setCarousel(carousel);
    }

    @Override
    public CarouselStateMachine getStateMachine() {
        return carouselStateMachine;
    }

    @Override
    public CarouselStateMachine.State getState() {
        return getStateMachine().getState();
    }

    @Override
    public void start() {
    }

    @Override
    public void stop() {
        getCarousel().setPower(0d);
    }

    @Override
    public void writeToTelemetry(Telemetry telemetry) {
    }

    @Override
    public void update(double dt) {
        getStateMachine().update(dt);
        getCarousel().setPower(getState().getSpeed());
    }

    @Override
    public String getName() {
        return "Carousel Subsystem";
    }

    private static void setCarouselStateMachine(CarouselStateMachine carouselStateMachine) {
        CarouselSubsystem.carouselStateMachine = carouselStateMachine;
    }

    private void setCarousel(RevMotor carousel){
        this.carousel = carousel;
    }
    private RevMotor getCarousel(){
        return carousel;
    }
}
