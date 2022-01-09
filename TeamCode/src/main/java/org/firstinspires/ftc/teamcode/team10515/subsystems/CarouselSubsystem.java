package org.firstinspires.ftc.teamcode.team10515.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.drivers.RevMotor;
import org.firstinspires.ftc.teamcode.team10515.states.CarouselStateMachine;

public class CarouselSubsystem implements ISubsystem<CarouselStateMachine, CarouselStateMachine.State> {
    private static CarouselStateMachine carouselStateMachine;
    private RevMotor carousel;
    private double kP = (1/5000d);
    private double output = 0d;

    public CarouselSubsystem(RevMotor carousel){//RevMotor leftFlywheel, RevMotor rightFlywheel) {
        setCarouselStateMachine(new CarouselStateMachine());
        setCarousel(carousel);
//        setLeftFlywheel(leftFlywheel);
//        setRightFlywheel(rightFlywheel);
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
        double error = getState().getSpeed() - getCarousel().getVelocity();
        output = kP * error;
        getStateMachine().update(dt);
        //getIntakeWheels().setPower(getState().getSpeed());
        getCarousel().setPower(output);
    }

    @Override
    public String getName() {
        return "Carousel Subsystem";
    }

    private static void setCarouselStateMachine(CarouselStateMachine carouselStateMachine) {
        CarouselSubsystem.carouselStateMachine = carouselStateMachine;
    }

    public double getOutput() {
        return output;
    }

    private void setCarousel(RevMotor carousel){
        this.carousel = carousel;
    }
    private RevMotor getCarousel(){
        return carousel;
    }
}
