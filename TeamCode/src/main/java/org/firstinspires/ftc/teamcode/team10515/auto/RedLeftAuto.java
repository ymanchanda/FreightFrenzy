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

@Autonomous(name = "Red Left", group = "XtremeV")
public class RedLeftAuto extends LinearOpMode {
    static final double COUNTS_PER_INCH = (383.6 * 0.5)/(3.77953 * 3.1415);
    FFBase drive;
    private static double dt;
    private static TimeProfiler updateRuntime;

    //ElapsedTime carouselTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    ElapsedTime waitTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    enum State {
        WAIT0,
        TRAJ1,
        TRAJ2,
        PICKUPLEFT, //bring the left dropper in pickup position
        CAROUSELRED, //spin left carousel
        WAIT1,
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
        drive.robot.getDropperRightSubsystem().getStateMachine().updateState(DropperRightStateMachine.State.INIT);
        drive.robot.getIntakeMotorSubsystem().getStateMachine().updateState(IntakeStateMachine.State.IDLE);

        Trajectory traj0 = drive.trajectoryBuilder(new Pose2d())
                .forward(1)
                .build();

        Trajectory traj1 = drive.trajectoryBuilder(traj0.end())
                .strafeLeft(1)
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .splineTo(new Vector2d(0, 0), Math.toRadians(45))
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
                    drive.followTrajectoryAsync(traj1);
                        currentState = State.TRAJ2;
                    waitTimer.reset();
                    }
                break;

                case TRAJ2:
                    if (!drive.isBusy()) {
                        drive.followTrajectoryAsync(traj2);
                        currentState = State.PICKUPLEFT;
                        waitTimer.reset();
                    }
                    break;

                case PICKUPLEFT:
                    //step 1 move dropper left down
                    drive.robot.getDropperLeftSubsystem().getStateMachine().updateState(DropperLeftStateMachine.State.PICKUP);
                    currentState = State.CAROUSELRED;
                    waitTimer.reset();
                    break;

                case CAROUSELRED:
                    //step 2 run carousel
                    drive.robot.getCarouselSubsystem().getStateMachine().updateState(CarouselStateMachine.State.RED);
                    currentState = State.WAIT1;
                    waitTimer.reset();
                    break;

                case WAIT1:
                    if (waitTimer.milliseconds() >= 4000) {
                        drive.robot.getCarouselSubsystem().getStateMachine().updateState(CarouselStateMachine.State.IDLE);
                        currentState = State.IDLE;
                        //drive.turn(Math.toRadians(9));
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