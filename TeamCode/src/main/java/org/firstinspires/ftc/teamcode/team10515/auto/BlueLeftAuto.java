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

@Autonomous(name = "Blue Left", group = "XtremeV")
public class BlueLeftAuto extends LinearOpMode {
    static final double COUNTS_PER_INCH = (383.6 * 0.5)/(3.77953 * 3.1415);
    FFBase drive;
    private static double dt;
    private static TimeProfiler updateRuntime;

    static final Vector2d Traj1 = new Vector2d(5.9,24);
    static final double angleForTraj1 = Math.toRadians(-90);
    static final Pose2d Traj2 = new Pose2d(-6,55, Math.toRadians(0));
    static final double angleForTraj2 = Math.toRadians(180);
    static final Vector2d Traj3 = new Vector2d(0,69.5);
    static final Vector2d Traj4 = new Vector2d(49, 69.5);

    //ElapsedTime carouselTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    ElapsedTime waitTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    enum State {
        TOHUB,
        BACKTOSTART,
        DROPRIGHT, //bring the right dropper in drop position
        PICKUPRIGHT, //bring the right dropper in pickup position
        TOPARK,
        IDLE
    }

    State currentState = State.IDLE;

    Pose2d startPose = new Pose2d(8.5, 63, Math.toRadians(-90));

    FFCV ffcv = new FFCV();
    boolean hasCVInit = false;
    String placement = "right";
    float confidence = 0;

    public void runOpMode() throws InterruptedException {
        setUpdateRuntime(new TimeProfiler(false));

        drive = new FFBase(hardwareMap);
        drive.setPoseEstimate(startPose);
        drive.robot.getElevSubsystem().getStateMachine().updateState(ElevStateMachine.State.IDLE);
        drive.robot.getCarouselSubsystem().getStateMachine().updateState(CarouselStateMachine.State.IDLE);
        drive.robot.getDropperLeftSubsystem().getStateMachine().updateState(DropperLeftStateMachine.State.INIT);
        drive.robot.getDropperRightSubsystem().getStateMachine().updateState(DropperRightStateMachine.State.PICKUP);
        drive.robot.getIntakeMotorSubsystem().getStateMachine().updateState(IntakeStateMachine.State.IDLE);

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(3)
                .splineTo(Traj1, angleForTraj1)
                .build();

        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(traj1.end())
                .splineToLinearHeading(Traj2, angleForTraj2)
                .lineTo(Traj3)
                .build();

        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(traj2.end())
                .lineTo(Traj4)
                .build();

        drive.getExpansionHubs().update(getDt());

        drive.robot.getElevSubsystem().update(getDt());
        drive.robot.getCarouselSubsystem().update(getDt());
        drive.robot.getDropperLeftSubsystem().update(getDt());
        drive.robot.getDropperRightSubsystem().update(getDt());
        drive.robot.getIntakeMotorSubsystem().update(getDt());

        waitForStart();

        if (isStopRequested()) return;

        ffcv.init(hardwareMap);
        while(ffcv.getFrameCount() == 0){
            telemetry.addData("Waiting", ffcv.getFrameCount());
        }
        telemetry.addLine("CV Init done");
        ffcv.recognize(true);
        placement = ffcv.getPlacement();
        confidence = ffcv.getConfidence();
        telemetry.addData("Placement: ", placement);
        telemetry.addData("Confidence: ", confidence);

        currentState = State.TOHUB;

        while (opModeIsActive() && !isStopRequested()) {

            setDt(getUpdateRuntime().getDeltaTime(TimeUnits.SECONDS, true));

            switch (currentState) {

                case TOHUB:
                    if (waitTimer.milliseconds() >= 100) {
                        drive.followTrajectorySequenceAsync(traj1);
                        if (placement == "left"){
                            drive.robot.getElevSubsystem().getStateMachine().updateState(ElevStateMachine.State.EXTENDMIDDLE);
                        }
                        else{
                            drive.robot.getElevSubsystem().getStateMachine().updateState(ElevStateMachine.State.EXTENDTOP);
                        }
                        currentState = State.DROPRIGHT;
                        waitTimer.reset();
                    }
                    break;

                case DROPRIGHT:
                    if (!drive.isBusy()){
                        drive.robot.getDropperRightSubsystem().getStateMachine().updateState(DropperRightStateMachine.State.DROPOFF);
                        currentState = State.PICKUPRIGHT;
                        waitTimer.reset();
                    }
                    break;

                case PICKUPRIGHT:
                    if(waitTimer.milliseconds() > 1000){
                        drive.robot.getDropperRightSubsystem().getStateMachine().updateState(DropperRightStateMachine.State.PICKUP);
                        drive.robot.getElevSubsystem().getStateMachine().updateState(ElevStateMachine.State.RETRACT);
                        currentState = State.BACKTOSTART;
                        waitTimer.reset();
                    }
                    break;

                case BACKTOSTART:
                    if (waitTimer.milliseconds() > 1000) {
                        drive.followTrajectorySequenceAsync(traj2);
                        currentState = State.TOPARK;
                        waitTimer.reset();
                    }
                    break;

                case TOPARK:
                    if(!drive.isBusy()){
                        drive.followTrajectorySequenceAsync(traj3);
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
