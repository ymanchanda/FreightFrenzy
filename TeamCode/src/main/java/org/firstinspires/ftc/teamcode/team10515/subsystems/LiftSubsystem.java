package org.firstinspires.ftc.teamcode.team10515.subsystems;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.lib.annotations.PIDSVA;
import org.firstinspires.ftc.teamcode.lib.control.ControlConstants;
import org.firstinspires.ftc.teamcode.lib.drivers.RevMotor;
import org.firstinspires.ftc.teamcode.lib.motion.IMotionProfile;
import org.firstinspires.ftc.teamcode.lib.motion.ResidualVibrationReductionMotionProfilerGenerator;
import org.firstinspires.ftc.teamcode.lib.util.Time;
import org.firstinspires.ftc.teamcode.lib.util.TimeProfiler;
import org.firstinspires.ftc.teamcode.lib.util.TimeUnits;
import org.firstinspires.ftc.teamcode.team10515.states.LiftStateMachine;

import java.util.function.DoubleSupplier;

@PIDSVA(name = "Extend",
        P = 0.4d,
        I = 0d,
        D = 0d,
        S = 0.14d,
        V = (1 - 0.14) / 15d,
        A = 0d
)

@PIDSVA(name = "Retract",
        P = 0.0025d,
        I = 0d,
        D = 0d,
        S = 0.06d,
        V = 1 / 55d,
        A = 0d
)
public class LiftSubsystem implements ISubsystem<LiftStateMachine, LiftStateMachine.State> {
    private static final Time STONE_IN_ROBOT_TIME_THRESHOLD = new Time(0.6d, TimeUnits.SECONDS);
    private static final double STONE_IN_ROBOT_DISTANCE_THRESHOLD = 4d;
    private static final ControlConstants EXTEND_CONTROL_CONSTANTS;
    private static final ControlConstants RETRACT_CONTROL_CONSTANTS;

    private static LiftStateMachine liftStateMachine;
    private RevMotor lift;

    private Rev2mDistanceSensor stoneDetector;
    private TimeProfiler timeProfilerStoneDetection;

    private static IMotionProfile extensionProfile = null;
    private static double setpoint = 0d;
    private static double desiredSetpoint = 0d;
    private double lastError;
    private double runningSum;

    private static DoubleSupplier manualControlExtension;


    private boolean isDeliveryMode = false;

    static {
        new Thread(ResidualVibrationReductionMotionProfilerGenerator::init).start();
        PIDSVA[] controllers = LiftSubsystem.class.getAnnotationsByType(PIDSVA.class);
        if(controllers.length == 2) {
            PIDSVA extendController;
            PIDSVA retractController;
            extendController  = controllers[0];
            retractController = controllers[0];

            EXTEND_CONTROL_CONSTANTS = new ControlConstants(
                extendController.P(), extendController.I(), extendController.D(),
                    extendController.S(), extendController.V(), extendController.A()
            );

            RETRACT_CONTROL_CONSTANTS = new ControlConstants(
                retractController.P(), retractController.I(), retractController.D(),
                    retractController.S(), retractController.V(), retractController.A()
            );
        } else {
            EXTEND_CONTROL_CONSTANTS  = new ControlConstants();
            RETRACT_CONTROL_CONSTANTS = new ControlConstants();
        }

        LiftStateMachine.setRunExtension((setpoint) -> {
            if(getExtensionProfile() != null) {
                getExtensionProfile().start();
                LiftSubsystem.setpoint = setpoint;
            }
        });
    }

    public LiftSubsystem(RevMotor lift) {
        setLiftStateMachine(new LiftStateMachine(this));
        setLift(lift);
        setTimeProfilerStoneDetection(new TimeProfiler(false));
        setLastError(0d);
        resetRunningSum();
    }

    public void resetRunningSum() {
        setRunningSum(0d);
    }

    @Override
    public LiftStateMachine getStateMachine() {
        return liftStateMachine;
    }

    @Override
    public LiftStateMachine.State getState() {
        return getStateMachine().getState();
    }

    @Override
    public void start() {
        //getStoneDetector().resetDeviceConfigurationForOpMode();
        //getTimeProfilerStoneDetection().start();
    }

    @Override
    public void stop() {
        getLift().setPower(0d);
        //Yogesh - commented close
        //getStoneDetector().close();
    }

    @Override
    public String getName() {
        return "Feeder";
    }

    @Override
    public void writeToTelemetry(Telemetry telemetry) {

    }

    @Override
    public void update(double dt) {
        if(getTimeProfilerStoneDetection().getDeltaTime(TimeUnits.SECONDS, false) > 200d) {
            getTimeProfilerStoneDetection().start();
        }

        getStateMachine().update(dt);
        getLiftStateMachine().update(dt);

        double error                = getSetpoint() - getLift().getPosition();
        double setpointVelocity     = 0d;
        double setpointAcceleration = 0d;
        if(getExtensionProfile() != null && !getExtensionProfile().isDone()) {
            setpointVelocity     = getExtensionProfile().getVelocity();
            setpointAcceleration = getExtensionProfile().getAcceleration();
        }

        setRunningSum(getRunningSum() + error * dt);
        double output = 0d;
        if(getManualControlExtension() != null ) {
            //Manual control
            output = getManualControlExtension().getAsDouble();
            if(output > 0d) {
                getLiftStateMachine().updateState(LiftStateMachine.State.EXTEND);
            }

            output += getExtendControlConstants().kS();
        } else if(getLiftStateMachine().getState().equals(LiftStateMachine.State.EXTEND)) {
            output = getExtendControlConstants().getOutput(dt, error, getLastError(), getRunningSum(), setpointVelocity, setpointAcceleration, false);
            output += getExtendControlConstants().kS();
            if (closeToSetpoint(1 / 4d)) {
                //getVirtualFourBarStateMachine().updateState(VirtualFourBarStateMachine.State.STACK);
            }
        } else {
            output = getRetractControlConstants().getOutput(dt, error, getLastError(), getRunningSum(), setpointVelocity, setpointAcceleration, true);
        }

        setLastError(error);

        final double kP = 0.001d;
        double relativeError = 0.0d;
        double relativeOutput = kP * relativeError;

        getLift().setPower(output);
    }

    public void extend() {
        resetRunningSum();
        setSetpoint(50);
    }

    public void retract() {
        resetRunningSum();
        setSetpoint(0d);
    }

    public boolean closeToSetpoint(double threshold) {
        return Math.abs(getSetpoint() - getLift().getPosition()) <= threshold;
    }

    public static LiftStateMachine getLiftStateMachine() {
        return liftStateMachine;
    }

    public static void setLiftStateMachine(LiftStateMachine liftStateMachine) {
        LiftSubsystem.liftStateMachine = liftStateMachine;
    }

    public RevMotor getLift() {
        return lift;
    }

    public void setLift(RevMotor leftExtension) {
        this.lift = lift;
    }

    public static double getSetpoint() {
        return setpoint;
    }

    public void setSetpoint(double setpoint) {
        setDesiredSetpoint(setpoint);
        if(setpoint != getSetpoint() && (getExtensionProfile() == null || getExtensionProfile().isDone())) {
            if(setpoint != 0d) {
                //Extending
                getLiftStateMachine().updateState(LiftStateMachine.State.EXTEND);
                //setExtensionProfile(getStackTracker().motionProfilerSetpoints(true));
            } else {
                //Retracting
                getLiftStateMachine().updateState(LiftStateMachine.State.RETRACT);
                //setExtensionProfile(getStackTracker().motionProfilerSetpoints(false));
                setExtensionProfile(new ResidualVibrationReductionMotionProfilerGenerator(
                        getLift().getPosition(), -getLift().getPosition(), 25d, 50d
                ));
            }
        }
    }

    public static IMotionProfile getExtensionProfile() {
        return extensionProfile;
    }

    public static void setExtensionProfile(IMotionProfile extensionProfile) {
        LiftSubsystem.extensionProfile = extensionProfile;
    }

    public static ControlConstants getExtendControlConstants() {
        return EXTEND_CONTROL_CONSTANTS;
    }

    public static ControlConstants getRetractControlConstants() {
        return RETRACT_CONTROL_CONSTANTS;
    }

    public double getLastError() {
        return lastError;
    }

    public void setLastError(double lastError) {
        this.lastError = lastError;
    }

    public double getRunningSum() {
        return runningSum;
    }

    public void setRunningSum(double runningSum) {
        this.runningSum = runningSum;
    }

    public TimeProfiler getTimeProfilerStoneDetection() {
        return timeProfilerStoneDetection;
    }

    public void setTimeProfilerStoneDetection(TimeProfiler timeProfilerStoneDetection) {
        this.timeProfilerStoneDetection = timeProfilerStoneDetection;
    }

    public static double getDesiredSetpoint() {
        return desiredSetpoint;
    }

    public static void setDesiredSetpoint(double desiredSetpoint) {
        LiftSubsystem.desiredSetpoint = desiredSetpoint;
    }

    public static DoubleSupplier getManualControlExtension() {
        return manualControlExtension;
    }

    public static void setManualControlExtension(DoubleSupplier manualControlExtension) {
        LiftSubsystem.manualControlExtension = manualControlExtension;
    }
}
