/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.commands.ExampleCommand;
import frc.robot.commands.Auto.SparkControlwDash;
import frc.robot.commands.Carousel.SpinCarousel;
import frc.robot.commands.Climber.SetClimbSpeed;
import frc.robot.commands.ColorSpinner.ColorSpinnerExtend;
import frc.robot.commands.ColorSpinner.ColorSpinnerPosition;
import frc.robot.commands.ColorSpinner.ColorSpinnerRetract;
import frc.robot.commands.ColorSpinner.ColorSpinnerRotation;
import frc.robot.commands.Drive.DriveWithJoysticks;
import frc.robot.commands.Intake.IntakeWithAxis;
import frc.robot.commands.Intake.SetIntakeSpeed;
import frc.robot.commands.Intake.TargetAndIntake;
import frc.robot.commands.Shooter.ShooterSparkControl;
import frc.robot.commands.Shooter.SparkShoot2;
import frc.robot.commands.Shooter.TriggerHood;
import frc.robot.commands.Intake.LimeLightIntake;
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
import lib.oi.LogitechF310;
import lib.oi.Thrustmaster;
import lib.turn.Turn;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

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
  private final Climber climber = new Climber();

  // **JOYSTICKS**
  LogitechF310 controller = new LogitechF310(Constants.ControllerConstants.USB_CONTROLLER);
  Thrustmaster leftStick = new Thrustmaster(Constants.ControllerConstants.USB_LEFT_STICK);
  Thrustmaster rightStick = new Thrustmaster(Constants.ControllerConstants.USB_RIGHT_STICK);

  // **COMMANDS**
  private final ExampleCommand exampleCmd = new ExampleCommand(m_exampleSubsystem);

  // DRIVE
  private final DriveWithJoysticks driveWithJoysticks = new DriveWithJoysticks(drive, leftStick, rightStick);
  private final Turn turn90 = new Turn(drive, 90, 5, Constants.AutoConstants.TURN_PARAMS);
  private final SparkControlwDash testingSparkTuning = new SparkControlwDash(drive, 24, 3);

  // INTAKE
  private final SetIntakeSpeed setIntakeSpeed = new SetIntakeSpeed(intake, Constants.IntakeConstants.SPEED);
  private final SetIntakeSpeed reverseIntakeSpeed = new SetIntakeSpeed(intake, -Constants.IntakeConstants.SPEED);
  private final IntakeWithAxis intakeWithAxis = new IntakeWithAxis(intake, controller);
  private final LimeLightIntake limeLightIntake = new LimeLightIntake(drive, myLimelight,
      Constants.IntakeConstants.LIME_KP, Constants.IntakeConstants.LIME_KI, Constants.IntakeConstants.LIME_KD,
      Constants.IntakeConstants.LIME_SPEED);
  private final TargetAndIntake targetAndIntake = new TargetAndIntake(drive, intake, myLimelight);

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

  // SHOOTER
  // private final ShooterSparkControl shooterSparkControl = new
  // ShooterSparkControl(shooter, 4000);
  private final SparkShoot2 shoot = new SparkShoot2(shooter, 4500);
  private final TriggerHood triggerHoodZero = new TriggerHood(shooter, 0);
  private final TriggerHood triggerHoodOne = new TriggerHood(shooter, 1);

  // CAROUSEL
  private final SpinCarousel spinCarousel = new SpinCarousel(carousel);

  // CLIMBER
  private final SetClimbSpeed setClimbSpeed = new SetClimbSpeed(climber);

  // **AUTO SWITCHES**
  private DigitalInput autoSwitch1, autoSwitch2, autoSwitch3, autoSwitch4;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    drive.setDefaultCommand(driveWithJoysticks);

    // Configure the button bindings
    configureButtonBindings();

    drive.resetEncoders();
    // drive.resetAngle();

    // Sets up auto stuff
    configAutos();
  }

  private void configureButtonBindings() {
    // COLOR SPINNER
    // leftStick.left.whenPressed(colorSpinnerRotation);
    // leftStick.right.whenPressed(colorSpinnerPosition);
    // rightStick.right.whileHeld(colorSpinnerExtend);
    // rightStick.left.whileHeld(colorSpinnerRetract);

    // SHOOTER
    leftStick.middle.whileHeld(shoot);
    leftStick.right.whileHeld(triggerHoodZero);
    rightStick.right.whileHeld(triggerHoodOne);
    leftStick.left.whileHeld(triggerHoodOne);

    // INTAKE
    controller.x.whileHeld(setIntakeSpeed);
    controller.lb.whileHeld(intakeWithAxis);

    leftStick.left.whileHeld(limeLightIntake);
    // leftStick.left.whileHeld(targetAndIntake);

    rightStick.middle.whileHeld(setIntakeSpeed);
    rightStick.trigger.whileHeld(reverseIntakeSpeed);

    // TURRET
    rightStick.left.whileHeld(limeLightTurret);
    leftStick.trigger.whileHeld(triggerTurretZero);
    rightStick.trigger.whileHeld(triggerTurretOne);

    // CLIMBER

    // AUTO
    // controller.a.whenHeld(turn90);
    controller.a.whenHeld(testingSparkTuning);

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

    // TODO generate trapezoidal profiles (if using)

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

  // TODO auto switches
  /*
   * private Command getAutoFromSwitches() {
   * 
   * }
   */

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * (runs in autoInit)
   * 
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    drive.resetEncoders();
    drive.resetAngle();

    return exampleCmd;
  }
}
