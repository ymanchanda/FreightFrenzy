package org.firstinspires.ftc.teamcode.team10515.auto;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.lib.drivers.RevMotor;
import org.firstinspires.ftc.teamcode.team10515.FreightFrenzyRobot;
import org.firstinspires.ftc.teamcode.team10515.states.CarouselStateMachine;
import org.firstinspires.ftc.teamcode.team10515.subsystems.Drive;

public class FFAutoMethods extends FreightFrenzyRobot {
    private BNO055IMU imu;
    private Orientation angles;
    private RevMotor LF;
    private RevMotor LR;
    private RevMotor RF;
    private RevMotor RR;
    private RevMotor Lift;
    private RevMotor Intake;
    private RevMotor Carousel;

    @Override
    public void init(){
        super.init();
        /*LF = getDrive().getFrontLeft();
        LR = getDrive().getBackLeft();
        RF = getDrive().getFrontRight();
        RR = getDrive().getBackRight();
        Lift = getElevSubsystem().getElevWheels();
        Intake = getIntakeMotorSubsystem().getIntakeWheels();
        Carousel = getCarouselSubsystem().getCarousel();*/
        LF = getMotors()[1];
        LR = getMotors()[0];
        RF = getMotors()[3];
        RR = getMotors()[2];
        Lift = getMotors()[4];
        Intake = getMotors()[5];
        Carousel = getMotors()[6];
    }

    @Override
    public void start(){
        super.start();
    }

    @Override
    public void loop(){
        super.loop();
    }

    @Override
    public void init_loop(){
        super.init_loop();
    }

    @Override
    public void stop(){
        super.stop();
    }
    
    public void stopMoving(){
        LF.setPower(0);
        RF.setPower(0);
        LR.setPower(0);
        RR.setPower(0);
    }

    public double getCurrentHeading() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double angle = angles.firstAngle;
        if (angle < 0) {
            angle = -angle;
        } else {
            angle = 360 - angle;
        }
        return angle;
    }

    public void turnRight(double power, double heading){
        double currHeading = getCurrentHeading();
        while(currHeading < heading){
            LF.setPower(power);
            RF.setPower(-power);
            LR.setPower(power);
            RR.setPower(-power);
            currHeading = getCurrentHeading();
        }
    }

    public void turnLeft(double power, double heading){
        double currHeading = getCurrentHeading();
        while(currHeading < heading){
            LF.setPower(-power);
            RF.setPower(power);
            LR.setPower(-power);
            RR.setPower(power);
            currHeading = getCurrentHeading();
        }
    }

    public void forward(double power, int position){
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LF.setTargetPosition(LF.getCurrentEncoderTicks() + position);
        LR.setTargetPosition(LR.getCurrentEncoderTicks() + position);
        RF.setTargetPosition(RF.getCurrentEncoderTicks() + position);
        RR.setTargetPosition(RR.getCurrentEncoderTicks() + position);

        LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        LF.setPower(power);
        LR.setPower(power);
        RF.setPower(power);
        RR.setPower(power);

        while(LF.getMotor().isBusy() && RF.getMotor().isBusy() && LR.getMotor().isBusy() && RR.getMotor().isBusy()){

        }
        stopMoving();

        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void backward(double power, int position){
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LF.setTargetPosition(LF.getCurrentEncoderTicks() + -position);
        LR.setTargetPosition(LR.getCurrentEncoderTicks() + -position);
        RF.setTargetPosition(RF.getCurrentEncoderTicks() + -position);
        RR.setTargetPosition(RR.getCurrentEncoderTicks() + -position);

        LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        LF.setPower(-power);
        LR.setPower(-power);
        RF.setPower(-power);
        RR.setPower(-power);

        while(LF.getMotor().isBusy() && RF.getMotor().isBusy() && LR.getMotor().isBusy() && RR.getMotor().isBusy()){

        }
        stopMoving();

        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void strafeLeft(double power, int position){
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LF.setTargetPosition(LF.getCurrentEncoderTicks() + -position);
        LR.setTargetPosition(LR.getCurrentEncoderTicks() + position);
        RF.setTargetPosition(RF.getCurrentEncoderTicks() + position);
        RR.setTargetPosition(RR.getCurrentEncoderTicks() + -position);

        LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        LF.setPower(-power);
        LR.setPower(power);
        RF.setPower(power);
        RR.setPower(-power);

        while(LF.getMotor().isBusy() && RF.getMotor().isBusy() && LR.getMotor().isBusy() && RR.getMotor().isBusy()){

        }
        stopMoving();

        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void strafeRight(double power, int position){
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LF.setTargetPosition(LF.getCurrentEncoderTicks() + position);
        LR.setTargetPosition(LR.getCurrentEncoderTicks() + -position);
        RF.setTargetPosition(RF.getCurrentEncoderTicks() + -position);
        RR.setTargetPosition(RR.getCurrentEncoderTicks() + position);

        LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        LF.setPower(power);
        LR.setPower(-power);
        RF.setPower(-power);
        RR.setPower(power);

        while(LF.getMotor().isBusy() && RF.getMotor().isBusy() && LR.getMotor().isBusy() && RR.getMotor().isBusy()){

        }
        stopMoving();

        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void runCarousel(String playingSide){
        if(playingSide.equals("blue")){
            getCarouselSubsystem().getStateMachine().updateState(CarouselStateMachine.State.BLUE);
        }
        else if(playingSide.equals("red")){
            Carousel.setPower(-0.6);
        }
    }
    public void stopCarousel(){
        Carousel.setPower(0);
    }
}
