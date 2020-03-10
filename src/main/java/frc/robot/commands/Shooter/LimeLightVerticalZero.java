/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import static frc.robot.Constants.ShooterConstants.*;
import lib.limelightLib.ControlMode.LedMode;

public class LimeLightVerticalZero extends CommandBase {
  private Limelight lime;
  private Shooter shooter;
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
  private double direct;

  /**
   * Creates a new LimeLightVerticalZero.
   */
  public LimeLightVerticalZero(Limelight limeSub, Shooter shooterSub, double _kP, double _kI, double _kD) {
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
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double integralActiveZone = HOOD_ACTIVE_ZONE;
    double ang = lime.getVertOffset();

    error = ang;

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
    shooter.setHoodSpeed(-speed);


    direct = shooter.getHoodEncoder();
    shooter.setDirect(direct);
    //System.out.println(error);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopHood();
    lime.setPipeline(3);
  }

  public double getDirect()  {
    return direct;
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(error <= 1.0 && error >= -1.0)  {
      return true;
    }
    else  {
      return false;
    }
  }
}
