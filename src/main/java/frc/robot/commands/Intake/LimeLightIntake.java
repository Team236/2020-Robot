package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Drive;
//import frc.robot.RobotContainer;

public class LimeLightIntake extends CommandBase {
  private double kP;
  private double kI;
  private double kD;
  private double proportional;
  private double integral;
  private double derivative;
  private double speed;
  private Limelight myLimelightLocal;
  private Drive drive;
  private double error;
  private double errorT;
  private double lastError;

  public LimeLightIntake(Drive driveSub, Limelight limeSub, double _kP, double _kI, double _kD, double _speed) {
    this.kP = _kP;
    this.kI = _kI;
    this.kD = _kD;
    this.myLimelightLocal = limeSub;
    this.drive = driveSub;
    this.speed = _speed;
    addRequirements(myLimelightLocal);
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    // RobotContainer.drive.navx.reset();
    drive.resetEncoders();
    // System.out.println("gyro drive init");
    myLimelightLocal.setPipeline(0);
  }

  @Override
  public void execute() {
    double leftSpeed = speed;
    double rightSpeed = speed;

    double integralActiveZone = Constants.IntakeConstants.I_ACTIVE_ZONE;
    double ang = myLimelightLocal.getLimeLight().getdegRotationToTarget();

    error = ang;

    if ((error < integralActiveZone && error > -integralActiveZone)) {
      errorT += error;
    } else {
      errorT = 0;
    }
    if (errorT > 50 / kI) {
      errorT = 50 / kI;
    }

    proportional = error * kP;
    integral = errorT * kI;
    derivative = (error - lastError) * kD;

    if (error > -1.00 && error < 1.00) {
      derivative = 0.00;
    }

    lastError = error;

    leftSpeed += (proportional + integral + derivative);
    rightSpeed -= (proportional + integral + derivative);

    if (error > -1.00 && error < 1.00) {
      drive.setLeftSpeed(-speed);
      drive.setRightSpeed(-speed);
    }

    drive.setLeftSpeed(-leftSpeed);
    drive.setRightSpeed(-rightSpeed);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  // @Override
  protected void end() {
    drive.stop();
  }

  // @Override
  protected void interrupted() {
    end();
  }
}
