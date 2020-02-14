/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class ShooterSparkControl extends CommandBase {

  private Shooter shooter;
  private double speed;

  // Used for tuning from dashboard (not needed when hard-coded)
  private double dash_kF, dash_kP, dash_kI, dash_KD;
  private double kF, kP, kI, kD;

  // Last recorded constants on cmd end()
  private double last_kF, last_kP, last_kI, last_kD;

  /**
   * Sets shooter velocity using spark max velocity control
   * @param speed Desired speed in RPM
   */
  public ShooterSparkControl(Shooter shooter, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    addRequirements(this.shooter);

    // DASHBOARD CONSTANTS
    kF = 0;
    kP = 0;
    kI = 0;
    kD = 0;
    // HARDCODED CONSTANTS
    // kF = Constants.ShooterConstants.kFF;
    // kP = Constants.ShooterConstants.kP;
    // kI = Constants.ShooterConstants.kI;
    // kD = Constants.ShooterConstants.kD;

    // Posts last constants to dashboard
    SmartDashboard.putNumber("Shooter Feed Fwd", last_kF);
    SmartDashboard.putNumber("Shooter k_P", last_kP);
    SmartDashboard.putNumber("Shooter k_I", last_kI);
    SmartDashboard.putNumber("Shooter k_D", last_kD);

    this.speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.resetEncoder();

    shooter.setP(kP);
    shooter.setI(kI);
    shooter.setD(kD);
    shooter.setFF(Constants.ShooterConstants.kFF);

    // HARDCODED CONSTANTS (obselete, now set above)
    // shooterSub.setP(Constants.ShooterConstants.kP);
    // shooterSub.setI(Constants.ShooterConstants.kI);
    // shooterSub.setD(Constants.ShooterConstants.kD);
    // shooterSub.setFF(Constants.ShooterConstants.kFF);

    shooter.setOutputRange();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // TODO try moving tuning logic to init (before set) - might allow eliminating setting again
    // comment these four lines when PID tuning done
    dash_kF = SmartDashboard.getNumber("Shooter Feed Fwd", 0);
    dash_kP = SmartDashboard.getNumber("Shooter k_P", 0);
    dash_kI = SmartDashboard.getNumber("Shooter k_I", 0);
    dash_KD = SmartDashboard.getNumber("Shooter k_D", 0);
    // comment these eight lines when PID tuning done
    // if ((dash_kF != kF)) {
      // shooter.setFF(dash_kF);
      // kF = dash_kF;
    // }
    SmartDashboard.putNumber("current s_ff", dash_kF);
    if ((dash_kP != kP)) {
      shooter.setP(dash_kP);
      kP = dash_kP;
    }
    SmartDashboard.putNumber("current s_pp", dash_kP);
    if ((dash_kI != kI)) {
      shooter.setI(dash_kI);
      kI = dash_kI;
    }
    SmartDashboard.putNumber("current s_ii", dash_kI);
    if ((dash_KD != kD)) {
      shooter.setP(dash_KD);
      kD = dash_KD;
    }
    SmartDashboard.putNumber("current s_dd", dash_KD);

    // Set rpm setpoint
    shooter.setSetPoint(speed);

    SmartDashboard.putNumber("shooter setpoint", speed);
    SmartDashboard.putNumber("actual speed", shooter.getVelocity());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stop();

    // comment these four lines when PID tuning done
    last_kF = SmartDashboard.getNumber("Shooter Feed Fwd", 0);
    last_kP = SmartDashboard.getNumber("Shooter k_P", 0);
    last_kI = SmartDashboard.getNumber("Shooter k_I", 0);
    last_kD = SmartDashboard.getNumber("Shooter k_D", 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
