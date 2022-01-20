package org.firstinspires.ftc.teamcode.team10515.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.team10515.odometery.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.team10515.states.CarouselStateMachine;
import org.firstinspires.ftc.teamcode.team10515.states.DropperLeftStateMachine;
import org.firstinspires.ftc.teamcode.team10515.states.DropperRightStateMachine;
import org.firstinspires.ftc.teamcode.team10515.states.ElevStateMachine;
import org.firstinspires.ftc.teamcode.team10515.states.IntakeStateMachine;

import org.firstinspires.ftc.teamcode.lib.util.TimeProfiler;
import org.firstinspires.ftc.teamcode.lib.util.TimeUnits;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Blue Right", group = "XtremeV")
public class BlueRightAuto extends LinearOpMode {
    static final double COUNTS_PER_INCH = (383.6 * 0.5)/(3.77953 * 3.1415);
    FFBase drive;
    private static double dt;
    private static TimeProfiler updateRuntime;

    static final int Traj1 = 3;
    static final int Turn1 = -90;
    static final int Traj2 = 26;
    static final Vector2d Traj3 = new Vector2d(-12,40);
    static final double angleForTraj3 = Math.toRadians(-180);
    static final Vector2d Traj4 = new Vector2d(-28, 60);
    static final double angleForTraj4 = Math.toRadians(-180);
    static final Pose2d Traj5 = new Pose2d(-55,40,Math.toRadians(-180));
    static final double angleForTraj5 = Math.toRadians(180);


    //ElapsedTime carouselTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    ElapsedTime waitTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    enum State {
        WAIT0,
        TOCAROUSEL,
        SPIN, //spin carousel
        TOHUB,
        DROPRIGHT, //bring the right dropper in drop position
        PICKUPRIGHT, //bring the right dropper in pickup position
        TOPARK,
        IDLE
    }

    State currentState = State.IDLE;

    Pose2d startPose = new Pose2d(-28, 63, Math.toRadians(-90));

    public void runOpMode() throws InterruptedException {
        setUpdateRuntime(new TimeProfiler(false));

        drive = new FFBase(hardwareMap);
        drive.setPoseEstimate(startPose);
        drive.robot.getElevSubsystem().getStateMachine().updateState(ElevStateMachine.State.IDLE);
        drive.robot.getCarouselSubsystem().getStateMachine().updateState(CarouselStateMachine.State.IDLE);
        drive.robot.getDropperLeftSubsystem().getStateMachine().updateState(DropperLeftStateMachine.State.PICKUP);
        drive.robot.getDropperRightSubsystem().getStateMachine().updateState(DropperRightStateMachine.State.INIT); //Changed from init to pickup
        drive.robot.getIntakeMotorSubsystem().getStateMachine().updateState(IntakeStateMachine.State.IDLE);

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPose)
                .forward(Traj1)
                .build();

        //90 degree rotation in between: drive.turn(Math.toRadians(90));

        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(traj1.end().plus(new Pose2d(0, 0, Math.toRadians(Traj1)))) //Have to add this because of the turn (last position is not traj0.end())
                .forward(Traj2)
                .build();

        //Spin carousel

        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(traj2.end())
                .splineToConstantHeading(Traj3, angleForTraj3)
                .build();

        //Drop pre-loaded stone

        //Pickup dropper

        TrajectorySequence traj4 = drive.trajectorySequenceBuilder(traj3.end())//Stop carousel
                .splineToConstantHeading(Traj4, angleForTraj4)
                .splineToLinearHeading(Traj5, angleForTraj5)
                .build();

        drive.getExpansionHubs().update(getDt());

        drive.robot.getElevSubsystem().update(getDt());
        drive.robot.getCarouselSubsystem().update(getDt());
        drive.robot.getDropperLeftSubsystem().update(getDt());
        drive.robot.getDropperRightSubsystem().update(getDt());
        drive.robot.getIntakeMotorSubsystem().update(getDt());

        waitForStart();

        if (isStopRequested()) return;

        currentState = State.WAIT0;

        while (opModeIsActive() && !isStopRequested()) {

            setDt(getUpdateRuntime().getDeltaTime(TimeUnits.SECONDS, true));

            switch (currentState) {

                case WAIT0:
                    if (waitTimer.milliseconds() >= 100) {
                        drive.followTrajectorySequenceAsync(traj1);
                        currentState = State.TOCAROUSEL;
                        waitTimer.reset();
                    }
                    break;

                case TOCAROUSEL:
                    if (!drive.isBusy()) {
                        drive.turn(Math.toRadians(Turn1)); //This is traj1
                        drive.followTrajectorySequenceAsync(traj2);
                        currentState = State.SPIN;
                        waitTimer.reset();
                    }
                    break;

                case SPIN:
//                    drive.robot.getDropperLeftSubsystem().getStateMachine().updateState(DropperLeftStateMachine.State.PICKUP);
                    if(!drive.isBusy()) {
                        drive.robot.getCarouselSubsystem().getStateMachine().updateState(CarouselStateMachine.State.BLUE);
                        currentState = State.TOHUB;
                        waitTimer.reset();
                    }
                    break;

                case TOHUB:
                    if(waitTimer.milliseconds() >= 2500) { //TODO: Spin for one second?
                        drive.robot.getCarouselSubsystem().getStateMachine().updateState(CarouselStateMachine.State.IDLE);
                        drive.followTrajectorySequenceAsync(traj3);
                        drive.robot.getElevSubsystem().getStateMachine().updateState(ElevStateMachine.State.EXTEND);
                        currentState = State.DROPRIGHT;
                        waitTimer.reset();
                    }
                    break;

                case DROPRIGHT:
                    if(!drive.isBusy()){
                        drive.robot.getDropperLeftSubsystem().getStateMachine().updateState(DropperLeftStateMachine.State.DROPOFF);
                        currentState = State.PICKUPRIGHT;
                        waitTimer.reset();
                    }
                    break;

                case PICKUPRIGHT:
                    if(waitTimer.milliseconds() > 1000){
                        drive.robot.getDropperLeftSubsystem().getStateMachine().updateState(DropperLeftStateMachine.State.PICKUP);
                        drive.robot.getElevSubsystem().getStateMachine().updateState(ElevStateMachine.State.RETRACT);
                        currentState = State.TOPARK;
                        waitTimer.reset();
                    }
                    break;

                case TOPARK:
                    if(waitTimer.milliseconds() > 1000) {
                        drive.followTrajectorySequenceAsync(traj4);
                        currentState = State.IDLE;
                        waitTimer.reset();
                    }
                    break;

                case IDLE:
                    break;
            }


            drive.update();

            //The following code ensure state machine updates i.e. parallel execution with drivetrain
            drive.getExpansionHubs().update(getDt());
            drive.robot.getElevSubsystem().update(getDt());
            drive.robot.getCarouselSubsystem().update(getDt());
            drive.robot.getDropperLeftSubsystem().update(getDt());
            drive.robot.getDropperRightSubsystem().update(getDt());
            drive.robot.getIntakeMotorSubsystem().update(getDt());

        }

        drive.setMotorPowers(0.0,0.0,0.0,0.0);
    }
    public static TimeProfiler getUpdateRuntime() {
        return updateRuntime;
    }

    public static void setUpdateRuntime(TimeProfiler updaRuntime) {
        updateRuntime = updaRuntime;
    }

    public static double getDt() {
        return dt;
    }

    public static void setDt(double pdt) {
        dt = pdt;
    }
}
