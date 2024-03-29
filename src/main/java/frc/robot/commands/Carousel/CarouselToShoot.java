/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Carousel;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Carousel;

public class CarouselToShoot extends CommandBase {

  private Carousel carousel;

  /**
   * Creates a new CarouselToShoot.
   */
  public CarouselToShoot(Carousel carousel) {
    this.carousel = carousel;
    addRequirements(this.carousel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    carousel.spinRoller();
    carousel.spinGreenWheel();
    carousel.setSpeed(Constants.CarouselConstants.SHOOT_SPEED);
    carousel.setToShootServos(Constants.CarouselConstants.POPPER_UP);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    carousel.stopGreenWheel();
    carousel.stopRoller();
    carousel.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
