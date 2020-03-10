/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Carousel;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Carousel;

public class PopperServo extends CommandBase {
  private Carousel carousel;
  private double position;
  /**
   * Creates a new PopperServo.
   */
  public PopperServo(Carousel carousel, double position) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.carousel = carousel;
    this.position = position;
    // addRequirements(this.carousel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    carousel.setToShootServos(position); // 0 is up, close to 1.0 is down 0.8, 0.2
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    carousel.spinGreenWheel();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    carousel.stopGreenWheel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
