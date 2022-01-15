package org.firstinspires.ftc.teamcode.team10515;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.teamcode.lib.drivers.Motor;
import org.firstinspires.ftc.teamcode.lib.drivers.RevMotor;
import org.firstinspires.ftc.teamcode.lib.drivers.RevServo;
import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.lib.util.TimeProfiler;
import org.firstinspires.ftc.teamcode.team10515.subsystems.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.team10515.subsystems.Drive;
import org.firstinspires.ftc.teamcode.team10515.subsystems.DropperLeftSubsystem;
import org.firstinspires.ftc.teamcode.team10515.subsystems.DropperRightSubsystem;
import org.firstinspires.ftc.teamcode.team10515.subsystems.ExpansionHubs;
//import org.firstinspires.ftc.teamcode.team10515.subsystems.FlickerSubsystem;
//import org.firstinspires.ftc.teamcode.team10515.subsystems.ForkliftSubsystem2;
import org.firstinspires.ftc.teamcode.team10515.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.team10515.subsystems.ElevSubsystem;
import org.firstinspires.ftc.teamcode.team10515.subsystems.RobotStateEstimator;
//import org.firstinspires.ftc.teamcode.team10515.subsystems.ShooterSubsystem;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.ExpansionHubServo;

import java.util.Arrays;

/**
 * Motor naming convention:
 *     Drivetrain
 *         Front Left Wheel -> LF
 *         Back Left Wheel   -> LR
 *         Front Right Wheel -> RF
 *         Back Right Wheel  -> RR
  *     Intake
 *         Intake Motor -> Intake
 *      lift
 *         Lift Motor -> Lift
 *     Outtake
 *         Left Arm Servo  -> Left
 *         Right Arm Servo -> Right
 *      Carousel
 *          Carousel Motor -> Carousel
 * Misc. sensors naming convention:

 */
public abstract class FreightFrenzyRobot extends Robot {
    //private  RevBlinkinLedDriver lights;
    private TimeProfiler matchRuntime;
    private ExpansionHubs expansionHubs;
    private RobotStateEstimator robotStateEstimator;
    private Drive drive;
    private ElevSubsystem elevSubsystem;
    private IntakeSubsystem intakeMotorSubsystem;
    private CarouselSubsystem carouselSubsystem;
    private DropperLeftSubsystem dropperLeftSubsystem;
    private DropperRightSubsystem dropperRightSubsystem;


    @Override
    public void init() {
        super.init();
        setExpansionHubs(new ExpansionHubs(this,
                hardwareMap.get(ExpansionHubEx.class, "Control Hub"),
                hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1"))
        );

        setMotors(new RevMotor[] {
                new RevMotor((ExpansionHubMotor)(hardwareMap.get("LR")), true, true, true, true, Motor.GOBILDA_435_RPM.getENCODER_TICKS_PER_REVOLUTION(), getWheelDiameter(), 2d),
                new RevMotor((ExpansionHubMotor)(hardwareMap.get("LF")), true, true, true, false, Motor.GOBILDA_435_RPM.getENCODER_TICKS_PER_REVOLUTION(), getWheelDiameter(), 2d),
                new RevMotor((ExpansionHubMotor)(hardwareMap.get("RR")), false, true, true, false, Motor.GOBILDA_435_RPM.getENCODER_TICKS_PER_REVOLUTION(), getWheelDiameter(), 2d),
                new RevMotor((ExpansionHubMotor)(hardwareMap.get("RF")), false, true, true, true, Motor.GOBILDA_435_RPM.getENCODER_TICKS_PER_REVOLUTION(), getWheelDiameter(), 2d),

                new RevMotor((ExpansionHubMotor)(hardwareMap.get("Intake")), true, false, false, false),//, Motor.GOBILDA_435_RPM.getENCODER_TICKS_PER_REVOLUTION()),// 50.8, 2d),
                new RevMotor((ExpansionHubMotor)(hardwareMap.get("Lift")), true, true, true, false, Motor.GOBILDA_312_RPM.getENCODER_TICKS_PER_REVOLUTION()),
                new RevMotor((ExpansionHubMotor)(hardwareMap.get("Carousel")), false, false, true, false), //Motor.GOBILDA_312_RPM.getENCODER_TICKS_PER_REVOLUTION()),
        });

        setServos(new RevServo[] {
                //new RevServo((ExpansionHubServo)(hardwareMap.get("Elevator Servo"))),
                new RevServo((ExpansionHubServo)(hardwareMap.get("Left"))),
                new RevServo((ExpansionHubServo)(hardwareMap.get("Right")))

        });

        setRobotStateEstimator(new RobotStateEstimator(this, hardwareMap.get(BNO055IMU.class, "imu"), new Pose2d()));
        setDrive(new Drive(getRobotStateEstimator(), getMotors()[0], getMotors()[1], getMotors()[2], getMotors()[3]));
        setIntakeMotorSubsystem(new IntakeSubsystem(getMotors()[4]));
        setElevSubsystem(new ElevSubsystem(getMotors()[5]));
        setCarouselSubsystem(new CarouselSubsystem(getMotors()[6]));

        setDropperLeftSubsystem(new DropperLeftSubsystem(getServos()[0]));
        setDropperRightSubsystem(new DropperRightSubsystem(getServos()[1]));
        setMatchRuntime(new TimeProfiler(false));
    }

    @Override
    public void init_loop() {
        super.init_loop();
    }

    @Override
    public void start() {
        super.start();
        getExpansionHubs().start();
        getDrive().start();
        Arrays.stream(getMotors()).forEach(RevMotor::resetEncoder);
        getMatchRuntime().start();
    }

    @Override
    public void loop() {
        super.loop();
        getExpansionHubs().update(getDt());
        getDrive().update(getDt());
        getIntakeMotorSubsystem().update(getDt());
        getElevSubsystem().update(getDt());
        getCarouselSubsystem().update(getDt());

        getDropperLeftSubsystem().update(getDt());
        getDropperRightSubsystem().update(getDt());
    }
//        getEndGameExtensionSubsystem().update(getDt());
//        if(getMatchRuntime().getDeltaTime(TimeUnits.SECONDS, false) >= 90d &&
//                getMatchRuntime().getDeltaTime(TimeUnits.SECONDS, false) <= 95d) {
//            //Set lights to give crazy patterns during endgame.
//            getLights().setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_PARTY_PALETTE);
//        } else if(Feeder.getFeederExtensionStateMachine().hasReachedStateGoal(FeederExtensionStateMachine.State.EXTEND)) {
//            getLights().setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
//        } else if(Feeder.getFeederStoneGripperStateMachine().hasReachedStateGoal(FeederStoneGripperStateMachine.State.GRIP)) {
//            getLights().setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
//        } else if(getFlywheels().getStateMachine().hasReachedStateGoal(FlywheelStateMachine.State.INTAKE)) {
//            getLights().setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
//        }
//    }

    @Override
    public void stop() {
        super.stop();
        getExpansionHubs().stop();
        getDrive().stop();
        getIntakeMotorSubsystem().stop();
        getElevSubsystem().stop();
        getCarouselSubsystem().stop();

        getDropperLeftSubsystem().stop();
        getDropperRightSubsystem().stop();

    }

    public ExpansionHubs getExpansionHubs() {
        return expansionHubs;
    }

    public void setExpansionHubs(ExpansionHubs expansionHubs) {
        this.expansionHubs = expansionHubs;
    }

    public RobotStateEstimator getRobotStateEstimator() {
        return robotStateEstimator;
    }

    public void setRobotStateEstimator(RobotStateEstimator robotStateEstimator) {
        this.robotStateEstimator = robotStateEstimator;
    }

    public Drive getDrive() {
        return drive;
    }

    public void setDrive(Drive drive) {
        this.drive = drive;
    }

    public IntakeSubsystem getIntakeMotorSubsystem() {
        return intakeMotorSubsystem;
    }

    public void setIntakeMotorSubsystem(IntakeSubsystem intakeMotorSubsystem){
        this.intakeMotorSubsystem = intakeMotorSubsystem;
    }

    public ElevSubsystem getElevSubsystem() {
        return elevSubsystem;
    }

    public void setElevSubsystem(ElevSubsystem elevSubsystem){
        this.elevSubsystem = elevSubsystem;
    }

    public CarouselSubsystem getCarouselSubsystem() {
        return carouselSubsystem;
    }

    public void setCarouselSubsystem(CarouselSubsystem carouselSubsystem){
        this.carouselSubsystem = carouselSubsystem;
    }
    /*

    public ShooterSubsystem getShooterSubsystem() { return shooterMotors; }

    public void setShooterSubsystem(ShooterSubsystem shooterMotors){ this.shooterMotors = shooterMotors; }
*/

    public DropperLeftSubsystem getDropperLeftSubsystem() {
        return dropperLeftSubsystem;
    }

    public void setDropperLeftSubsystem(DropperLeftSubsystem dropperLeftSubsystem) {
        this.dropperLeftSubsystem = dropperLeftSubsystem;
    }

    public DropperRightSubsystem getDropperRightSubsystem() {
        return dropperRightSubsystem;
    }

    public void setDropperRightSubsystem(DropperRightSubsystem dropperRightSubsystem) {
        this.dropperRightSubsystem = dropperRightSubsystem;
    }

/*    public FlickerSubsystem getFlickerSubsystem(){
        return flickerSubsystem;
    }

    public void setFlickerSubsystem(FlickerSubsystem flickerSubsystem){
        this.flickerSubsystem = flickerSubsystem;
    }


    public ForkliftSubsystem2 getForkliftSubsystem2() {
        return forkliftSubsystem;
    }

    public void setForkliftSubsystem2(ForkliftSubsystem2 forkliftSubsystem){
        this.forkliftSubsystem = forkliftSubsystem;
    }*/
    /*public  RevBlinkinLedDriver getLights() {
        return lights;
    }

    public void setLights(RevBlinkinLedDriver lights) {
        this.lights = lights;
    }*/

//    public void setElevatorSensor(Rev2mDistanceSensor range) {
//        this.elevatorSensor = range;
//    }

    public TimeProfiler getMatchRuntime() {
        return matchRuntime;
    }

    public void setMatchRuntime(TimeProfiler matchRuntime) {
        this.matchRuntime = matchRuntime;
    }

    public Pose2d getRobotPose() {
        return getRobotStateEstimator().getPose();
    }

    public double getRobotSpeed() {
        return getRobotStateEstimator().getVelocityPose().getTranslation().norm() +
                Math.abs(getRobotStateEstimator().getVelocityPose().getRotation().getRadians());
    }
}
