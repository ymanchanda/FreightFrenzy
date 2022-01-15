package org.firstinspires.ftc.teamcode.team10515.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

    static final int Traj0 = 3;
    static final int Traj1 = -90;
    static final int Traj2 = 26;
    static final int Traj3 = 42;
    static final int Traj4 = 21;
    static final int Traj5 = 42;

    //ElapsedTime carouselTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    ElapsedTime waitTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    enum State {
        WAIT0,
        TRAJ1,
        TRAJ2,
        SPIN, //spin carousel
        TRAJ3,
        TRAJ4,
        DROPRIGHT, //bring the right dropper in drop position
        PICKUPRIGHT, //bring the right dropper in pickup position
        TRAJ5,
        IDLE
    }

    State currentState = State.IDLE;

    //Pose2d startPose = new Pose2d(-62.375, -15, Math.toRadians(0));

    public void runOpMode() throws InterruptedException {
        setUpdateRuntime(new TimeProfiler(false));

        drive = new FFBase(hardwareMap);

        drive.robot.getElevSubsystem().getStateMachine().updateState(ElevStateMachine.State.IDLE);
        drive.robot.getCarouselSubsystem().getStateMachine().updateState(CarouselStateMachine.State.IDLE);
        drive.robot.getDropperLeftSubsystem().getStateMachine().updateState(DropperLeftStateMachine.State.PICKUP);
        drive.robot.getDropperRightSubsystem().getStateMachine().updateState(DropperRightStateMachine.State.INIT); //Changed from init to pickup
        drive.robot.getIntakeMotorSubsystem().getStateMachine().updateState(IntakeStateMachine.State.IDLE);

        Trajectory traj0 = drive.trajectoryBuilder(new Pose2d())
                .forward(Traj0)
                .build();

        //90 degree rotation in between: drive.turn(Math.toRadians(90));

        Trajectory traj2 = drive.trajectoryBuilder(traj0.end().plus(new Pose2d(0, 0, Math.toRadians(Traj1)))) //Have to add this because of the turn (last position is not traj0.end())
                .forward(Traj2)
                .build();

        //Spin carousel

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .back(Traj3)
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())//Stop carousel
                .strafeLeft(Traj4)
                .build();

        //Drop pre-loaded stone

        //Pickup dropper

        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .forward(Traj5)
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
                        drive.followTrajectoryAsync(traj0);
                        currentState = State.TRAJ1;
                        waitTimer.reset();
                    }
                    break;

                case TRAJ1:
                    if (!drive.isBusy()) {
                        drive.turn(Math.toRadians(Traj1)); //This is traj1
                        currentState = State.TRAJ2;
                        waitTimer.reset();
                    }
                    break;

                case TRAJ2:
                    if (!drive.isBusy()) {
                        drive.followTrajectoryAsync(traj2);
                        currentState = State.SPIN;
                        waitTimer.reset();
                    }
                    break;

                case SPIN:
//                    drive.robot.getDropperLeftSubsystem().getStateMachine().updateState(DropperLeftStateMachine.State.PICKUP);
                    if(!drive.isBusy()) {
                        drive.robot.getCarouselSubsystem().getStateMachine().updateState(CarouselStateMachine.State.BLUE);
                        currentState = State.TRAJ3;
                        waitTimer.reset();
                    }
                    break;

                case TRAJ3:
                    if(waitTimer.milliseconds() >= 3000) { //TODO: Spin for one second?
                        drive.robot.getCarouselSubsystem().getStateMachine().updateState(CarouselStateMachine.State.IDLE);
                        drive.followTrajectoryAsync(traj3);
                        currentState = State.TRAJ4;
                        waitTimer.reset();
                    }
                    break;

                case TRAJ4:
                    if (!drive.isBusy()) {
                        drive.followTrajectoryAsync(traj4);
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
                        currentState = State.TRAJ5;
                        waitTimer.reset();
                    }
                    break;

                case TRAJ5:
                    if(waitTimer.milliseconds() > 1000) {
                        drive.followTrajectoryAsync(traj5);
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
