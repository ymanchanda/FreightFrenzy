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

@Autonomous(name = "Red Left Defense", group = "XtremeV")
public class RedLeftDefenseAuto extends LinearOpMode {
    static final double COUNTS_PER_INCH = (383.6 * 0.5)/(3.77953 * 3.1415);
    FFBase drive;
    private static double dt;
    private static TimeProfiler updateRuntime;

    static final Vector2d Traj1 = new Vector2d(-24,-23);
    static final double angleforTraj1 = Math.toRadians(90);
    static final Vector2d Traj2 = new Vector2d(-54,-23);

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

    Pose2d startPose = new Pose2d(-28,-63, Math.toRadians(90));
    public void runOpMode() throws InterruptedException {
        setUpdateRuntime(new TimeProfiler(false));

        drive = new FFBase(hardwareMap);
        drive.setPoseEstimate(startPose);
        drive.robot.getElevSubsystem().getStateMachine().updateState(ElevStateMachine.State.IDLE);
        drive.robot.getCarouselSubsystem().getStateMachine().updateState(CarouselStateMachine.State.IDLE);
        drive.robot.getDropperLeftSubsystem().getStateMachine().updateState(DropperLeftStateMachine.State.INIT);
        drive.robot.getDropperRightSubsystem().getStateMachine().updateState(DropperRightStateMachine.State.PICKUP); //Changed from init to pickup
        drive.robot.getIntakeMotorSubsystem().getStateMachine().updateState(IntakeStateMachine.State.IDLE);

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .splineTo(Traj1, angleforTraj1)
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .strafeTo(Traj2)
                .build();


        drive.getExpansionHubs().update(getDt());

        drive.robot.getElevSubsystem().update(getDt());
        drive.robot.getCarouselSubsystem().update(getDt());
        drive.robot.getDropperLeftSubsystem().update(getDt());
        drive.robot.getDropperRightSubsystem().update(getDt());
        drive.robot.getIntakeMotorSubsystem().update(getDt());

        waitForStart();

        if (isStopRequested()) return;

        currentState = State.TOHUB;

        while (opModeIsActive() && !isStopRequested()) {

            setDt(getUpdateRuntime().getDeltaTime(TimeUnits.SECONDS, true));

            switch (currentState) {


                case TOHUB:
                    if (!drive.isBusy()){
                        drive.followTrajectoryAsync(traj1);
                        drive.robot.getElevSubsystem().getStateMachine().updateState(ElevStateMachine.State.EXTENDTOP);
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
                        currentState = State.TOPARK;
                        waitTimer.reset();
                    }
                    break;

                case TOPARK:
                    if (waitTimer.milliseconds() > 1000) {
                        drive.followTrajectoryAsync(traj2);
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
