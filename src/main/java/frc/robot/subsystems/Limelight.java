/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lib.limelightLib.TheLimeLight;
/**
 * Add your docs here.
 */
public class Limelight extends SubsystemBase {

  private TheLimeLight limelight;

  private Limelight lime;
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

  public Limelight() {
    limelight = new TheLimeLight("limelight");
  }

  //@Override
  public void initDefaultCommand() {
      //setDefaultCommand(new )
  }

  public TheLimeLight getLimeLight() {
    return limelight;
  }

  public boolean isTarget() {
    return getLimeLight().getIsTargetFound();
  }

  public double getAngleOffset() {
    return getLimeLight().getdegRotationToTarget();
  }

  public double getVertOffset() {
    return getLimeLight().getdegVerticalToTarget();
  }

  public double getTargetArea() {
    return getLimeLight().getTargetArea();
  }

  public double getPipelineLatency() {
    return getLimeLight().getPipelineLatency();
  }

  public void setPipeline(Integer pipeline) {
    getLimeLight().setPipeline(pipeline);
  }

  public boolean getIsTargetFound() {
    return getLimeLight().getIsTargetFound();
  }

  public double doPID(Limelight limeSub, double _kP, double _kI, double _kD, double integralZone)
  {
    double integralActiveZone = integralZone;
    double ang = lime.getLimeLight().getdegRotationToTarget();

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

    return speed;
  }
}

