/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.commands.ExampleCommand;
import frc.robot.commands.Auto.SparkControlwDash;
import frc.robot.commands.Carousel.CarouselToShoot;
import frc.robot.commands.Carousel.PopperServo;
import frc.robot.commands.Carousel.SpinCarousel;
import frc.robot.commands.Climber.ClimberWithAxis;
import frc.robot.commands.Climber.RelayControl;
import frc.robot.commands.Climber.SetClimbSpeed;
import frc.robot.commands.ColorSpinner.ColorSpinnerExtend;
import frc.robot.commands.ColorSpinner.ColorSpinnerPosition;
import frc.robot.commands.ColorSpinner.ColorSpinnerRetract;
import frc.robot.commands.ColorSpinner.ColorSpinnerRotation;
import frc.robot.commands.ColorSpinner.ExtendCSgroup;
import frc.robot.commands.ColorSpinner.RetractCSgroup;
import frc.robot.commands.Drive.DriveWithJoysticks;
import frc.robot.commands.Drive.ClimberVision;
import frc.robot.commands.Intake.IntakeWithAxis;
import frc.robot.commands.Intake.SetIntakeSpeed;
import frc.robot.commands.Intake.TargetAndIntake;
import frc.robot.commands.Shooter.ShooterSparkControl;
import frc.robot.commands.Shooter.SparkShoot2;
import frc.robot.commands.Shooter.TriggerHood;
import frc.robot.commands.Shooter.LimeLightVerticalZero;
import frc.robot.commands.Shooter.LimeLightHoodOffset;
import frc.robot.commands.Shooter.LimeSequentialShooter;
import frc.robot.commands.Shooter.CombinedShoot;
import frc.robot.commands.Intake.LimeLightIntake;
import frc.robot.commands.Intake.RaiseLowerIntake;
import frc.robot.commands.Turret.LimeLightTurret;
import frc.robot.commands.Turret.TriggerTurret;
import frc.robot.subsystems.Carousel;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ColorSpinner;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Limelight;
import lib.commands.TimeoutCommand;
import lib.commands.Wait;
import static frc.robot.Constants.TurretConstants.*;
import static frc.robot.Constants.ShooterConstants.*;
import lib.motionProfile.TrapProfile;
import lib.oi.LogitechF310;
import lib.oi.Thrustmaster;
import lib.oi.triggers.JoystickPOV;
import lib.oi.triggers.JoystickPOV.Direction;
import lib.turn.Turn;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // **SUBSYSTEMS**
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Drive drive = new Drive();
  private final Intake intake = new Intake();
  private final Shooter shooter = new Shooter();
  private final Carousel carousel = new Carousel();
  private final Turret turret = new Turret();
  private final ColorSpinner colorSpinner = new ColorSpinner();
  public final static Limelight myLimelight = new Limelight();
  //public final static Limelight myLimelightIntake = new Limelight("intakeLL");
  private final Climber climber = new Climber();

  // **JOYSTICKS**
  LogitechF310 controller = new LogitechF310(Constants.ControllerConstants.USB_CONTROLLER);
  Thrustmaster leftStick = new Thrustmaster(Constants.ControllerConstants.USB_LEFT_STICK);
  Thrustmaster rightStick = new Thrustmaster(Constants.ControllerConstants.USB_RIGHT_STICK);

  // **COMMANDS**
  private final ExampleCommand exampleCmd = new ExampleCommand(m_exampleSubsystem);

  // DRIVE
  private final DriveWithJoysticks driveWithJoysticks = new DriveWithJoysticks(drive, leftStick, rightStick, false);
  private final DriveWithJoysticks cubeWithJoysticks = new DriveWithJoysticks(drive, leftStick, rightStick, true);
  private final Turn turn90 = new Turn(drive, 90, 3, Constants.AutoConstants.TURN_PARAMS);
  private final Turn turn45 = new Turn(drive, 45, 3, Constants.AutoConstants.TURN_PARAMS);
  private final SparkControlwDash testingSparkTuning = new SparkControlwDash(drive, 24, 3);

  private final ClimberVision climberVision = new ClimberVision(myLimelight, turret, shooter);
  // public TrapProfile testProfile;

  // INTAKE
  private final SetIntakeSpeed setIntakeSpeed = new SetIntakeSpeed(intake, Constants.IntakeConstants.SPEED);
  private final SetIntakeSpeed reverseIntakeSpeed = new SetIntakeSpeed(intake, -Constants.IntakeConstants.SPEED);
  private final IntakeWithAxis intakeWithAxis = new IntakeWithAxis(intake, controller);
  /*private final LimeLightIntake limeLightIntake = new LimeLightIntake(drive, myLimelightIntake,
      Constants.IntakeConstants.LIME_KP, Constants.IntakeConstants.LIME_KI, Constants.IntakeConstants.LIME_KD,
      Constants.IntakeConstants.LIME_SPEED);*/
  private final TargetAndIntake targetAndIntake = new TargetAndIntake(drive, intake, myLimelight);
  private final RaiseLowerIntake raiseIntake = new RaiseLowerIntake(intake, true);
  private final RaiseLowerIntake lowerIntake = new RaiseLowerIntake(intake, false);

  // TURRET
  private final LimeLightTurret limeLightTurret = new LimeLightTurret(myLimelight, turret,
      Constants.TurretConstants.TURRET_kP, Constants.TurretConstants.TURRET_kI, Constants.TurretConstants.TURRET_kD);
  private final TriggerTurret triggerTurretZero = new TriggerTurret(turret, 0);
  private final TriggerTurret triggerTurretOne = new TriggerTurret(turret, 1);

  // COLOR SPINNER
  private final ColorSpinnerRotation colorSpinnerRotation = new ColorSpinnerRotation(colorSpinner);
  private final ColorSpinnerPosition colorSpinnerPosition = new ColorSpinnerPosition(colorSpinner);
  private final ColorSpinnerExtend colorSpinnerExtend = new ColorSpinnerExtend(colorSpinner);
  private final ColorSpinnerRetract colorSpinnerRetract = new ColorSpinnerRetract(colorSpinner);
  private final ExtendCSgroup extendCSgroup = new ExtendCSgroup(colorSpinner);
  private final RetractCSgroup retractCSgroup = new RetractCSgroup(colorSpinner);

  // SHOOTER
  // private final ShooterSparkControl shooterSparkControl = new
  // ShooterSparkControl(shooter, 4000);
  // private final SparkShoot2 shoot = new SparkShoot2(shooter, 3000);//4500
  private final SparkShoot2 shoot1 = new SparkShoot2(shooter, 4000);// 4500

  private final SparkShoot2 shoot = new SparkShoot2(shooter, 4500);
  private final LimeLightVerticalZero limeLightVerticalZero = new LimeLightVerticalZero(myLimelight, shooter, HOOD_kP,
      HOOD_kI, HOOD_kD);
  private final LimeLightHoodOffset limeLightHoodOffset = new LimeLightHoodOffset(myLimelight, shooter, HOOD_kP,
      HOOD_kI, HOOD_kD);
  private final LimeSequentialShooter limeSequentialShooter = new LimeSequentialShooter(shooter, myLimelight, HOOD_kP,
      HOOD_kI, HOOD_kD);
  private final CombinedShoot combinedShoot = new CombinedShoot(shooter, myLimelight, turret, HOOD_kP, HOOD_kI, HOOD_kD,
      TURRET_kP, TURRET_kI, TURRET_kD);
  // HOOD
  private final TriggerHood triggerHoodUp = new TriggerHood(shooter, 0);
  private final TriggerHood triggerHoodDown = new TriggerHood(shooter, 1);

  // CAROUSEL
  private final SpinCarousel spinCarousel = new SpinCarousel(carousel, false);
  private final SpinCarousel spinCarousel2 = new SpinCarousel(carousel, false);
  private final SpinCarousel revCarousel = new SpinCarousel(carousel, true);
  private final CarouselToShoot feed = new CarouselToShoot(carousel);
  private final PopperServo popperServoUp = new PopperServo(carousel, Constants.CarouselConstants.POPPER_UP);
  private final PopperServo popperServoDown = new PopperServo(carousel, .8);
  private final TimeoutCommand carouselFor2 = new TimeoutCommand(spinCarousel, .5);

  // INTAKE/CAROUSEL GROUPS
  // private final ParallelCommandGroup waitThenShoot = new
  // ParallelCommandGroup(new Wait(seconds))
  private final ParallelCommandGroup shootSeq = new ParallelCommandGroup(feed, shoot1);
  // TODO carousel doesn't run on intake currently
  // private final ParallelCommandGroup intakeAndCarousel = new
  // ParallelCommandGroup(spinCarousel, setIntakeSpeed);
  private final ParallelCommandGroup intakeAndCarousel = new ParallelCommandGroup(setIntakeSpeed);

  // CLIMBER
  // private final SetClimbSpeed climbFwd = new SetClimbSpeed(climber,
  // Constants.ClimberConstants.SPEED);
  private final ClimberWithAxis climberWithAxis = new ClimberWithAxis(climber, controller);
  private final RelayControl disengageRelay = new RelayControl(climber, true);
  private final RelayControl engageRelay = new RelayControl(climber, false);

  // **AUTO SWITCHES**
  private DigitalInput autoSwitch1, autoSwitch2, autoSwitch3, autoSwitch4;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    drive.setDefaultCommand(driveWithJoysticks);

    // Configure the button bindings
    configureButtonBindings();

    // Sets up auto stuff
    configAutos();

    // Sets LimeLight LEDs to off

  }

  private void configureButtonBindings() {
    // DRIVE
    leftStick.left.whileHeld(cubeWithJoysticks);
    JoystickButton climberVisionBtn = new JoystickButton(rightStick, 8);
    climberVisionBtn.whileHeld(climberVision);

    // INTAKE
    rightStick.trigger.whileHeld(intakeAndCarousel);
    rightStick.middle.whileHeld(reverseIntakeSpeed);
    // TODO intake and limelight options
    JoystickPOV raiseIntBtn = new JoystickPOV(controller, Direction.UP);
    raiseIntBtn.whenHeld(raiseIntake);
    JoystickPOV lowerIntBtn = new JoystickPOV(controller, Direction.DOWN);
    lowerIntBtn.whenHeld(lowerIntake);

    // SHOOTER
    leftStick.trigger.whileHeld(shootSeq);

    rightStick.six.whileHeld(limeLightVerticalZero);
    rightStick.seven.whileHeld(combinedShoot);

    // TURRET
    JoystickPOV turretLeftBtn = new JoystickPOV(leftStick, Direction.LEFT);
    turretLeftBtn.whileHeld(triggerTurretOne);
    JoystickPOV turretRightBtn = new JoystickPOV(leftStick, Direction.RIGHT);
    turretRightBtn.whileHeld(triggerTurretZero);

    rightStick.five.whileHeld(limeLightTurret);

    // HOOD
    JoystickPOV hoodUpBtn = new JoystickPOV(leftStick, Direction.UP);
    hoodUpBtn.whileHeld(triggerHoodUp);
    JoystickPOV hoodDownBtn = new JoystickPOV(leftStick, Direction.DOWN);
    hoodDownBtn.whileHeld(triggerHoodDown);

    // CAROUSEL
    leftStick.middle.whileHeld(revCarousel);
    rightStick.right.whenPressed(carouselFor2);
    rightStick.left.whileHeld(spinCarousel2);

    // FEEDER
    JoystickPOV popperUpBtn = new JoystickPOV(rightStick, Direction.UP);
    popperUpBtn.whileHeld(popperServoUp);
    JoystickPOV popperDownBtn = new JoystickPOV(rightStick, Direction.DOWN);
    popperDownBtn.whileHeld(popperServoDown);

    // COLOR SPINNER
    controller.y.whenHeld(colorSpinnerExtend);
    controller.a.whenHeld(colorSpinnerRetract);
    controller.x.whenHeld(colorSpinnerRotation);
    controller.b.whenHeld(colorSpinnerPosition);

    // CLIMBER
    controller.rb.whenPressed(disengageRelay);
    controller.lb.whenPressed(engageRelay);
    controller.back.whileHeld(climberWithAxis);

  }

  /**
   * Call this in robotContainer constructor (to run during robotInit)
   */
  private void configAutos() {
    // TODO create auto switches
    autoSwitch1 = new DigitalInput(Constants.AutoConstants.DIO_SWITCH_1);
    autoSwitch2 = new DigitalInput(Constants.AutoConstants.DIO_SWITCH_2);
    autoSwitch3 = new DigitalInput(Constants.AutoConstants.DIO_SWITCH_3);
    autoSwitch4 = new DigitalInput(Constants.AutoConstants.DIO_SWITCH_4);

    // private final SequentialCommandGroup shootMove = new
    // SequentialCommandGroup(shoot, testingSparkTuning);

    // TODO generate trapezoidal profiles (if using)
    // testProfile = new TrapProfile(-24, 100, 100, 0, 3);
  }

  public void doInPeriodic() {

    try {
      SmartDashboard.putBoolean("switch1", autoSwitch1.get());
      SmartDashboard.putBoolean("switch2", autoSwitch2.get());
      SmartDashboard.putBoolean("switch3", autoSwitch3.get());
      SmartDashboard.putBoolean("switch4", autoSwitch4.get());
    } catch (Exception e) {
      System.out.println("switches bad");
    }

  }

  /**
   * Called in robotInit() of Robot.java
   */
  public void doOnInit() {
    drive.resetAngle();
    drive.resetEncoders();

    climber.relayOn();

  }

  /**
   * Called in disabledInit() of Robot.java
   */
  public void doOnDisable() {
    climber.relayOff();
  }

  public void relayOnDisable() {
    climber.relayOn();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * (runs in autoInit)
   * 
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // TODO determine auto from switches
    drive.resetEncoders();
    drive.resetAngle();

    return exampleCmd;
  }
}
