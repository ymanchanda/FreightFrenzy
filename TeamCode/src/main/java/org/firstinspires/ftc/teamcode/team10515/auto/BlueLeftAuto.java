package org.firstinspires.ftc.teamcode.team10515.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
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

@Autonomous(name = "Blue Left", group = "XtremeV")
public class BlueLeftAuto extends LinearOpMode {
    static final double COUNTS_PER_INCH = (383.6 * 0.5)/(3.77953 * 3.1415);
    FFBase drive;
    private static double dt;
    private static TimeProfiler updateRuntime;

    static final int Traj0 = 40;
    static final int Traj1 = 6;
    static final int Traj2 = 30;
    static final int Traj3 = 90; //Angle
    static final int Traj4 = 12;
    static final int Traj5 = 40;

    //ElapsedTime carouselTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    ElapsedTime waitTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    enum State {
        WAIT0,
        TRAJ1,
        DROPRIGHT,
        PICKUPRIGHT,
        TRAJ2,
        TRAJ3,
        TRAJ4,
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
        drive.robot.getDropperLeftSubsystem().getStateMachine().updateState(DropperLeftStateMachine.State.INIT);
        drive.robot.getDropperRightSubsystem().getStateMachine().updateState(DropperRightStateMachine.State.PICKUP);
        drive.robot.getIntakeMotorSubsystem().getStateMachine().updateState(IntakeStateMachine.State.IDLE);

        Trajectory traj0 = drive.trajectoryBuilder(new Pose2d())
                .forward(Traj0)
                .build();

        Trajectory traj1 = drive.trajectoryBuilder(traj0.end())
                .strafeRight(Traj1)
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .back(Traj2)
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj2.end().plus(new Pose2d(0, 0, Math.toRadians(Traj3))))
                .strafeLeft(Traj4)
                .build();

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
                        drive.robot.getElevSubsystem().getStateMachine().updateState(ElevStateMachine.State.EXTEND);
                        currentState = State.TRAJ1;
                        waitTimer.reset();
                    }
                    break;

                case TRAJ1:
                    if (!drive.isBusy()){
                        drive.followTrajectoryAsync(traj1);
                        currentState = State.DROPRIGHT;
                        waitTimer.reset();
                    }

                case DROPRIGHT:
                    if(!drive.isBusy()){
                        drive.robot.getDropperRightSubsystem().getStateMachine().updateState(DropperRightStateMachine.State.DROPOFF);
                        currentState = State.PICKUPRIGHT;
                        waitTimer.reset();
                    }
                    break;

                case PICKUPRIGHT:
                    if(waitTimer.milliseconds() > 1000){
                        drive.robot.getDropperRightSubsystem().getStateMachine().updateState(DropperRightStateMachine.State.PICKUP);
                        drive.robot.getElevSubsystem().getStateMachine().updateState(ElevStateMachine.State.RETRACT);
                        currentState = State.TRAJ2;
                        waitTimer.reset();
                    }
                    break;

                case TRAJ2:
                    if (!drive.isBusy()) {
                        drive.followTrajectoryAsync(traj2);
                        currentState = State.TRAJ3;
                        waitTimer.reset();
                    }
                    break;

                case TRAJ3:
                    if(!drive.isBusy()){
                        drive.turn(Math.toRadians(Traj3));
                        currentState = State.TRAJ4;
                        waitTimer.reset();
                    }
                    break;

                case TRAJ4:
                    if (!drive.isBusy()) {
                        drive.followTrajectoryAsync(traj4);
                        currentState = State.TRAJ5;
                        waitTimer.reset();
                    }
                    break;

                case TRAJ5:
                    if (!drive.isBusy()) {
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
