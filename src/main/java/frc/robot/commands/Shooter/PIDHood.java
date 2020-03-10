/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import static frc.robot.Constants.ShooterConstants.*;

public class PIDHood extends CommandBase {
  
  private Shooter hood;

  private double kP;
  private double kI;
  private double kD;
  private double set;
  private double error;
  private double errorT;
  private double lastError;
  private double proportional;
  private double integral;
  private double derivative;
  private double position;
  private int integralActiveZone;
  private double speed;
  /**
   * Creates a new PIDHood.
   */
  public PIDHood(Shooter shooterSub, double set, double _kP, double _kI, double _kD) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.hood = shooterSub;
    this.kP = _kP;
    this.kI = _kI;
    this.kD = _kD;
    this.set = set;

    addRequirements(hood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    integralActiveZone = HOOD_PID_ACTIVE_ZONE;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    position = hood.getHoodEncoder();
    error = set - position;

    //Proportional
    proportional = error * kP;

    //Integral
    if((error < integralActiveZone && error > -integralActiveZone))
    {
      errorT += error;
    }
    else 
    {
      errorT = 0;
    }
    if(errorT > 50 / kI)
    {
      errorT = 50 / kI;
    }

    integral = errorT * kI;

    //Derivative
    derivative = (error - lastError) * kD;

    if(error == 0.0)
    {
      derivative = 0.0;
    }
    
    lastError = error;

    speed = (proportional + integral + derivative);
    //negative -= (proportional + integral + derivative);

    hood.setHoodSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hood.stopHood();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
