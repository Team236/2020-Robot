/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import static frc.robot.Constants.ShooterConstants.*;
import frc.robot.commands.Shooter.LimeLightVerticalZero;
import frc.robot.RobotContainer;

public class LimeLightHoodOffset extends CommandBase {
  private Limelight lime;
  private Shooter shooter;
  private double direct;
  private double offset;
  private double error;
  private double proportional;
  private double integral;
  private double derivative;
  private double errorT;
  private double lastError;
  private double speed;
  private double kP;
  private double kI;
  private double kD;
  /**
   * Creates a new LimeLightHoodOffset.
   */
  public LimeLightHoodOffset(Limelight limeSub, Shooter shooterSub, double _kP, double _kI, double _kD) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.lime = limeSub;
    this.shooter = shooterSub;
    this.kP = _kP;
    this.kI = _kI;
    this.kD = _kD;

    addRequirements(shooter); 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lime.setPipeline(0);
    //Get direct angle thru encoder
    direct = shooter.getDirect();
    //Calculate offset
    offset = (direct);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double integralActiveZone = HOOD_ACTIVE_ZONE;
    error = offset - shooter.getHoodEncoder();

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


    shooter.setHoodRaw(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopHood();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(error <= 1.5)  {
      return true;
    }
    else  {
      return false;
    }
  }
}
