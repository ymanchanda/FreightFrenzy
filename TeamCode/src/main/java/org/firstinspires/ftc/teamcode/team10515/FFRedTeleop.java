package org.firstinspires.ftc.teamcode.team10515;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.team10515.states.CarouselStateMachine;
import org.firstinspires.ftc.teamcode.team10515.states.DropperLeftStateMachine;
import org.firstinspires.ftc.teamcode.team10515.states.DropperRightStateMachine;
import org.firstinspires.ftc.teamcode.team10515.states.ElevStateMachine;
import org.firstinspires.ftc.teamcode.team10515.states.IntakeStateMachine;
import org.firstinspires.ftc.teamcode.team10515.subsystems.LiftSubsystem;

/*
 * This {@code class} acts as the driver-controlled program for FTC team 10515 for the Skystone
 * challenge. By extending {@code SkystoneRobot}, we already have access to all the robot subsystems,
 * so only tele-operated controls need to be defined here.
 *
 * The controls for this robot are:
 *  User 1:
 *      Drive:
 *          Left & Right joysticks     -> Mecanum drive
 *          Left-stick-button          -> Robot speed to default value
 *          Right-stick-button         -> Disable/enable movement
 *          Left-Trigger               -> Decrease robot speed
 *          Right-Trigger              -> Increase robot speed
 *      Forklift:
 *          A-Button (pressed)         -> Lower forklift
 *          Y-Button (pressed)         -> Higher forklift
 *  User 2:
 *      Flywheel Intake:
 *          Left bumper (pressed)      -> Stop Intake
 *          Right bumper (pressed)     -> Start Intake
 *      Shooter:
 *          Left-trigger               -> Stop shoooter
 *          Right-trigger              -> Start shooter
 *          A-button (pressed)         -> Hit ring into launch position (servo)
 *          Y-button (pressed)         -> Toggle shooter
 *      Shooter Speed:
 *          Dpad-right                 -> Shooter speed to 1
 *          Dpad-down                  -> Shooter speed to 2
 *          Dpad-left                  -> Shooter speed to 3
 *          Dpad-up                    -> Shooter speed to 4
 *
 * @see UltimateGoalRobot
 */

@TeleOp(name = "Red Tele-Op", group = "Main")
public class FFRedTeleop extends FreightFrenzyRobot {

    public double currentTime = 0; // keep track of current time
    private boolean dropperLeft = true;
    private boolean stopintake = true;
    private boolean liftdown = true;
//    public double previousTime = 0; // keep track of last time A was pressed (Flicker was moved)
//    public double flickerInterval = 1; // after 1 second has passed since pressing A, move Flicker back to original position

//    public boolean shooterIsOn = false;
//    public ShooterStateMachine.State currentShooterSpeed = ShooterStateMachine.State.SPEED1;

//    public boolean allowMovement = true;

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        super.loop();

        setDrivetrainPower(new Pose2d(gamepad1.left_stick_y, gamepad1.left_stick_x, new Rotation2d(-gamepad1.right_stick_x, false)));
        //telemetry.addData("left x: ", gamepad1.left_stick_x);
        //telemetry.addData("left y: ", gamepad1.left_stick_y);

        //-----------------------------------------------------------------------------------------------------------------------------------------
        //Gamepad 1

        if(getEnhancedGamepad1().getLeft_trigger() > 0){
            getCarouselSubsystem().getStateMachine().updateState(CarouselStateMachine.State.BLUE);
            telemetry.addLine("Pad1 Left Trigger");

        }
        else if(getEnhancedGamepad1().getRight_trigger() > 0){
            getCarouselSubsystem().getStateMachine().updateState(CarouselStateMachine.State.RED);
            telemetry.addLine("Pad1 Right Trigger");
        }
        else if(getEnhancedGamepad1().isBack()){
            getCarouselSubsystem().getStateMachine().updateState(CarouselStateMachine.State.IDLE);
            telemetry.addLine("Pad1 Back button");
        }

        if(getEnhancedGamepad1().isbJustPressed()){
            //allow intake to work because one of the droppers is in Pickup state but don't start the intake automatically
            stopintake = false;
            if (dropperLeft) {
                getDropperLeftSubsystem().getStateMachine().updateState(DropperLeftStateMachine.State.PICKUP);
                telemetry.addLine("Left Dropper: " + getDropperLeftSubsystem().getStateMachine().getState());
            }
            else {
                getDropperRightSubsystem().getStateMachine().updateState(DropperRightStateMachine.State.PICKUP);
                telemetry.addLine("Right Dropper: " + getDropperRightSubsystem().getStateMachine().getState());
            }
        }
        if(getEnhancedGamepad1().isxJustPressed()){
            stopintake = true;
           if (dropperLeft) {
                getDropperLeftSubsystem().getStateMachine().updateState(DropperLeftStateMachine.State.DROPOFF);
                telemetry.addLine("Left Dropper: " + getDropperLeftSubsystem().getStateMachine().getState());
            }
            else {
                getDropperRightSubsystem().getStateMachine().updateState(DropperRightStateMachine.State.DROPOFF);
                telemetry.addLine("Right Dropper: " + getDropperRightSubsystem().getStateMachine().getState());
            }
        }
        if (getEnhancedGamepad1().isDpadUpJustPressed()){
            stopintake = true;
            if (dropperLeft) {
                getDropperLeftSubsystem().getStateMachine().updateState(DropperLeftStateMachine.State.INIT);
                telemetry.addLine("Left Dropper: " + getDropperLeftSubsystem().getStateMachine().getState());
            }
            else {
                getDropperRightSubsystem().getStateMachine().updateState(DropperRightStateMachine.State.INIT);
                telemetry.addLine("Right Dropper: " + getDropperRightSubsystem().getStateMachine().getState());
            }
        }

        //-----------------------------------------------------------------------------------------------------------------------------------------
        //Gamepad 2

        if(getEnhancedGamepad2().getLeft_trigger() > 0){
            getIntakeMotorSubsystem().getStateMachine().updateState(IntakeStateMachine.State.OUTTAKE);
            telemetry.addLine("Pad2 Left Trigger");
        }
        else if(getEnhancedGamepad2().getRight_trigger() > 0){
            if (!stopintake && liftdown)
                getIntakeMotorSubsystem().getStateMachine().updateState(IntakeStateMachine.State.INTAKE);
            telemetry.addLine("Pad2 Right Trigger");
        }
        else if(getEnhancedGamepad2().isBack()){
            getIntakeMotorSubsystem().getStateMachine().updateState(IntakeStateMachine.State.IDLE);
            telemetry.addLine("Pad2 Back button");
        }

        if(getEnhancedGamepad2().isaJustPressed()){
            getElevSubsystem().getStateMachine().updateState(ElevStateMachine.State.RETRACT);
            liftdown = true;
//            getIntakeMotorSubsystem().getStateMachine().updateState(IntakeStateMachine.State.INTAKE);
            telemetry.addLine("a pressed lift up: " + getElevSubsystem().getStateMachine().getState());
        }

        if(getEnhancedGamepad2().isyJustPressed()){
            getElevSubsystem().getStateMachine().updateState(ElevStateMachine.State.EXTEND);
            stopintake = true;
            liftdown = false;
            telemetry.addLine("y pressed lift down: " + getElevSubsystem().getStateMachine().getState());
        }

        //Gamepad 2 decides which dropper is active i.e. Left or Right. Starts with Left as default.
        if(getEnhancedGamepad2().isDpadLeftJustPressed()){
            dropperLeft = true;
            stopintake = true;
            getDropperRightSubsystem().getStateMachine().updateState(DropperRightStateMachine.State.INIT);
            getDropperLeftSubsystem().getStateMachine().updateState(DropperLeftStateMachine.State.INIT);
        }

        if(getEnhancedGamepad2().isDpadRightJustPressed()){
            dropperLeft = false;
            stopintake = true;
            getDropperLeftSubsystem().getStateMachine().updateState(DropperLeftStateMachine.State.INIT);
            getDropperRightSubsystem().getStateMachine().updateState(DropperRightStateMachine.State.INIT);
        }

        if (stopintake) {
            //stop intake because droppers are not in pickup state
            getIntakeMotorSubsystem().getStateMachine().updateState(IntakeStateMachine.State.IDLE);
        }

        //--------------------------------------------------------------------------------------------------------------------------------
        // last year

        //Update flywheel intake
/*
        if(getEnhancedGamepad2().getRight_trigger()>0) {
            getFlywheels().getStateMachine().updateState(FlywheelStateMachine.State.INTAKE);
        } else if(getEnhancedGamepad2().getLeft_trigger() > 0) {
            getFlywheels().getStateMachine().updateState(FlywheelStateMachine.State.OUTTAKE);
        } else if(gamepad2.back) {
            getFlywheels().getStateMachine().updateState(FlywheelStateMachine.State.IDLE);
        }

        //Update Stack tracker
        if(getEnhancedGamepad2().isDpadRightJustPressed()) {
            getStackTracker().addStoneToStack();
        } else if(getEnhancedGamepad2().isDpadLeftJustPressed()) {
            getStackTracker().removeStoneFromStack();
        } else if(gamepad1.left_trigger > 0.05d) {
            getStackTracker().resetStack();
        }

        //Update feeder
        if(getEnhancedGamepad2().isyJustPressed()) {
            getFeeder().extend();
        } else if(getEnhancedGamepad2().isaJustPressed()) {
            getFeeder().retract();
        }

        //Toggle virtual four-bar
        if(getEnhancedGamepad2().isLeftBumperJustPressed()) {
            getFeeder().toggleVirtualFourBar();
        }

        //Check to release grip of stone for stacking
        if(getEnhancedGamepad2().isRight_bumper() */
/*&& getStackTracker().getExtensionHeight() == Feeder.getSetpoint()*//*
) {
            Feeder.getFeederStoneGripperStateMachine().updateState(FeederStoneGripperStateMachine.State.NO_GRIP);
            //  getFeeder().toggleStoneGripper();
        }

        if(getEnhancedGamepad2().isStart()){
            Feeder.getFeederStoneGripperStateMachine().updateState(FeederStoneGripperStateMachine.State.GRIP);
        }

        if(getEnhancedGamepad1().isB()){
            getFeeder().setDeliveryMode(true);
        }else if(getEnhancedGamepad1().isX()){
            getFeeder().setDeliveryMode(false);
        }

        //Toggle end game extension blocker to extend slides
        if(getEnhancedGamepad1().isBack()) {
            getEndGameExtensionSubsystem().getStateMachine().updateState(EndGameExtensionStateMachine.State.RELEASE_SLIDES);
        }

        if(getEnhancedGamepad2().isDpadDownJustPressed()) {
            getFoundationSubsystem().getStateMachine().updateState(FoundationStateMachine.State.GRAB);
        } else if(getEnhancedGamepad2().isDpadUpJustPressed()) {
            getFoundationSubsystem().getStateMachine().updateState(FoundationStateMachine.State.INIT);
        }

        if(getEnhancedGamepad1().isA()) {
            Feeder.getFlickerStateMachine().updateState(FlickerStateMachine.State.DROP);
        } else if(getEnhancedGamepad1().isY()) {
            Feeder.getFlickerStateMachine().updateState(FlickerStateMachine.State.HOLD);
        }


        telemetry.addLine("Stones stacked: " + getStackTracker());
        telemetry.addLine("Stacked Height: " + getStackTracker().getExtensionHeight());
        telemetry.addLine("Extension Setpoint: " + Feeder.getSetpoint());
        telemetry.addLine("Extension Desired Setpoint: " + Feeder.getDesiredSetpoint());
        telemetry.addLine("Extension Height: " + getFeeder().getLeftExtension().getPosition());
        telemetry.addLine("Extension State: " + Feeder.getFeederExtensionStateMachine().getState().getName());
        telemetry.addLine("Left Extension Power: " + getFeeder().getLeftExtension().getLastPower());
        telemetry.addLine("Right Extension Power: " + getFeeder().getRightExtension().getLastPower());
        telemetry.addLine("V4B State: " + Feeder.getVirtualFourBarStateMachine().getState().getName());
        telemetry.addLine("Time seen stone: " + getFeeder().getTimeProfilerStoneDetection().getDeltaTime(TimeUnits.SECONDS, false));
        telemetry.addLine("Stone distance: " + getFeeder().getStoneDetector().getDistance(DistanceUnit.INCH));
        telemetry.addLine("Feeder Extension Constants: " + Feeder.getExtendControlConstants());
        telemetry.addLine("Extension close to setpoint: " + getFeeder().closeToSetpoint(1 / 4d));
        telemetry.addLine("Extension Profile: " + (Feeder.getExtensionProfile() != null));
        if(Feeder.getExtensionProfile() != null) {
            telemetry.addLine("" + Feeder.getExtensionProfile().getPosition());
        }
*/

        currentTime = getRuntime();

    }
}
