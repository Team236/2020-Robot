/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Turret;

public class LimeLightTurret extends CommandBase {
  private Limelight lime;
  private Turret turret;
  private double kP;
  private double kI;
  private double kD;
  private double proportional;
  private double integral;
  private double derivative;
  private double speed;
  private double error;
  private double errorT;
  private double lastError;

  /**
   * Creates a new LimeLightShooter.
   */
  public LimeLightTurret(Limelight limeSub, Turret turretSub, double _kP, double _kI, double _kD) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.lime = limeSub;
    this.turret = turretSub;
    this.kP = _kP;
    this.kI = _kI;
    this.kD = _kD;

    addRequirements(lime);
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lime.setPipeline(0);
    turret.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double integralActiveZone = Constants.TurretConstants.I_ACTIVE_ZONE;
    double ang = lime.getLimeLight().getdegRotationToTarget();

    error = ang;

    // Proportional
    proportional = error * kP;

    // Integral
    if ((error < integralActiveZone && error > -integralActiveZone)) {
      errorT += error;
    } else {
      errorT = 0;
    }
    if (errorT > 50 / kI) {
      errorT = 50 / kI;
    }

    integral = errorT * kI;

    // Derivative
    derivative = (error - lastError) * kD;

    if (error == 0.0) {
      derivative = 0.0;
    }

    lastError = error;

    speed = (proportional + integral + derivative);
    // negative -= (proportional + integral + derivative);

    turret.setTurretSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
