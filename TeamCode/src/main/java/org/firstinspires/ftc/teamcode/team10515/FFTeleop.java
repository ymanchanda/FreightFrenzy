package org.firstinspires.ftc.teamcode.team10515;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.team10515.auto.FFBase;
import org.firstinspires.ftc.teamcode.team10515.odometery.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.team10515.states.CarouselStateMachine;
import org.firstinspires.ftc.teamcode.team10515.states.DropperLeftStateMachine;
import org.firstinspires.ftc.teamcode.team10515.states.DropperRightStateMachine;
import org.firstinspires.ftc.teamcode.team10515.states.ElevStateMachine;
import org.firstinspires.ftc.teamcode.team10515.states.IntakeStateMachine;

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

@TeleOp(name = "Main Tele-Op", group = "Main")
public class FFTeleop extends FreightFrenzyTeleopRobot {

    public double currentTime = 0; // keep track of current time
    private boolean dropperLeft = true;
    private boolean stopintake = true;
    private boolean liftdown = true;
//    public double previousTime = 0; // keep track of last time A was pressed (Flicker was moved)
//    public double flickerInterval = 1; // after 1 second has passed since pressing A, move Flicker back to original position

//    public boolean shooterIsOn = false;
//    public ShooterStateMachine.State currentShooterSpeed = ShooterStateMachine.State.SPEED1;

//    public boolean allowMovement = true;
    enum Mode {
        DRIVER_CONTROL,
        AUTOMATIC_CONTROL
    }
    Mode currentMode = Mode.DRIVER_CONTROL;

    Pose2d allianceRedHubPosition = new Pose2d(-12,-40, Math.toRadians(0));

    @Override
    public void init(){
        drive = new FFBase(hardwareMap, true);
        drive.setPoseEstimate(PoseStorage.currentPose);
        super.init();
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        super.loop();
        drive.update();
        Pose2d poseEstimate = drive.getPoseEstimate();
        switch (currentMode) {
            case DRIVER_CONTROL:
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y * 1,
                                -gamepad1.left_stick_x * 0.6,
                                -gamepad1.right_stick_x * 0.65
                        )
                );
/*
                if (getEnhancedGamepad1().isDpadLeftJustPressed()) {
                    // If the A button is pressed on gamepad1, we generate a splineTo()
                    // trajectory on the fly and follow it
                    // We switch the state to AUTOMATIC_CONTROL

                    TrajectorySequence allianceRedHub = drive.trajectorySequenceBuilder(poseEstimate)
                            .splineTo(new Vector2d(allianceRedHubPosition.getX(), allianceRedHubPosition.getY()), allianceRedHubPosition.getHeading())
                            .build();

                    drive.followTrajectorySequenceAsync(allianceRedHub);
                    currentMode = Mode.AUTOMATIC_CONTROL;
                }
                break;
 */
            case AUTOMATIC_CONTROL:
                // If x is pressed, we break out of the automatic following
                if (getEnhancedGamepad1().isyJustPressed()) {
                    drive.cancelFollowing();
                    currentMode = Mode.DRIVER_CONTROL;
                }

                // If drive finishes its task, cede control to the driver
                if (!drive.isBusy()) {
                    currentMode = Mode.DRIVER_CONTROL;
                }
                break;
        }

        //telemetry.addData("left x: ", gamepad1.left_stick_x);
        //telemetry.addData("left y: ", gamepad1.left_stick_y);

        //-----------------------------------------------------------------------------------------------------------------------------------------
        //Gamepad 1

        if(getEnhancedGamepad1().getLeft_trigger() > 0){
            drive.robot.getCarouselSubsystem().getStateMachine().updateState(CarouselStateMachine.State.BLUE);
            telemetry.addLine("Pad1 Left Trigger");

        }
        else if(getEnhancedGamepad1().getRight_trigger() > 0){
            drive.robot.getCarouselSubsystem().getStateMachine().updateState(CarouselStateMachine.State.RED);
            telemetry.addLine("Pad1 Right Trigger");
        }
        else if(getEnhancedGamepad1().isBack()){
            drive.robot.getCarouselSubsystem().getStateMachine().updateState(CarouselStateMachine.State.IDLE);
            telemetry.addLine("Pad1 Back button");
        }

        if(getEnhancedGamepad1().isbJustPressed()){
            //allow intake to work because one of the droppers is in Pickup state but don't start the intake automatically
            stopintake = false;
            if (dropperLeft) {
                drive.robot.getDropperLeftSubsystem().getStateMachine().updateState(DropperLeftStateMachine.State.PICKUP);
                telemetry.addLine("Left Dropper: " + drive.robot.getDropperLeftSubsystem().getStateMachine().getState());
            }
            else {
                drive.robot.getDropperRightSubsystem().getStateMachine().updateState(DropperRightStateMachine.State.PICKUP);
                telemetry.addLine("Right Dropper: " + drive.robot.getDropperRightSubsystem().getStateMachine().getState());
            }
        }
        if(getEnhancedGamepad1().isxJustPressed()){
            stopintake = true;
           if (dropperLeft) {
               drive.robot.getDropperLeftSubsystem().getStateMachine().updateState(DropperLeftStateMachine.State.DROPOFF);
                telemetry.addLine("Left Dropper: " + drive.robot.getDropperLeftSubsystem().getStateMachine().getState());
            }
            else {
               drive.robot.getDropperRightSubsystem().getStateMachine().updateState(DropperRightStateMachine.State.DROPOFF);
                telemetry.addLine("Right Dropper: " + drive.robot.getDropperRightSubsystem().getStateMachine().getState());
            }
        }
        if (getEnhancedGamepad1().isDpadUpJustPressed()){
            stopintake = true;
            if (dropperLeft) {
                drive.robot.getDropperLeftSubsystem().getStateMachine().updateState(DropperLeftStateMachine.State.INIT);
                telemetry.addLine("Left Dropper: " + drive.robot.getDropperLeftSubsystem().getStateMachine().getState());
            }
            else {
                drive.robot.getDropperRightSubsystem().getStateMachine().updateState(DropperRightStateMachine.State.INIT);
                telemetry.addLine("Right Dropper: " + drive.robot.getDropperRightSubsystem().getStateMachine().getState());
            }
        }

        //-----------------------------------------------------------------------------------------------------------------------------------------
        //Gamepad 2

        if(getEnhancedGamepad2().getLeft_trigger() > 0){
            drive.robot.getIntakeMotorSubsystem().getStateMachine().updateState(IntakeStateMachine.State.OUTTAKE);
            telemetry.addLine("Pad2 Left Trigger");
        }
        else if(getEnhancedGamepad2().getRight_trigger() > 0){
            if (!stopintake && liftdown)
                drive.robot.getIntakeMotorSubsystem().getStateMachine().updateState(IntakeStateMachine.State.INTAKE);
            telemetry.addLine("Pad2 Right Trigger");
        }
        else if(getEnhancedGamepad2().isBack()){
            drive.robot.getIntakeMotorSubsystem().getStateMachine().updateState(IntakeStateMachine.State.IDLE);
            telemetry.addLine("Pad2 Back button");
        }

        if(getEnhancedGamepad2().isaJustPressed()){
            drive.robot.getElevSubsystem().getStateMachine().updateState(ElevStateMachine.State.RETRACT);
            liftdown = true;
//            getIntakeMotorSubsystem().getStateMachine().updateState(IntakeStateMachine.State.INTAKE);
            telemetry.addLine("a pressed lift up: " + drive.robot.getElevSubsystem().getStateMachine().getState());
        }

        if(getEnhancedGamepad2().isyJustPressed()){
            drive.robot.getElevSubsystem().getStateMachine().updateState(ElevStateMachine.State.EXTENDTOP);
            stopintake = true;
            liftdown = false;
            telemetry.addLine("y pressed lift down: " + drive.robot.getElevSubsystem().getStateMachine().getState());
        }

        //Gamepad 2 decides which dropper is active i.e. Left or Right. Starts with Left as default.
        if(getEnhancedGamepad2().isDpadLeftJustPressed()){
            dropperLeft = true;
            stopintake = true;
            drive.robot.getDropperRightSubsystem().getStateMachine().updateState(DropperRightStateMachine.State.INIT);
            drive.robot.getDropperLeftSubsystem().getStateMachine().updateState(DropperLeftStateMachine.State.INIT);
        }

        if(getEnhancedGamepad2().isDpadRightJustPressed()){
            dropperLeft = false;
            stopintake = true;
            drive.robot.getDropperLeftSubsystem().getStateMachine().updateState(DropperLeftStateMachine.State.INIT);
            drive.robot.getDropperRightSubsystem().getStateMachine().updateState(DropperRightStateMachine.State.INIT);
        }

        if (stopintake) {
            //stop intake because droppers are not in pickup state
            drive.robot.getIntakeMotorSubsystem().getStateMachine().updateState(IntakeStateMachine.State.IDLE);
        }

        currentTime = getRuntime();

    }
}
