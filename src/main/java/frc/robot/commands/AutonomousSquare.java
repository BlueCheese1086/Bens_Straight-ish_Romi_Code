// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

public class AutonomousSquare extends InstantCommand {
  /** Creates a new AutonomousSquare. */
  Drivetrain drivetrain;
  public AutonomousSquare(Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SequentialCommandGroup scg = new SequentialCommandGroup(new DriveDistance(0.8, 12, drivetrain), new TurnDegrees(0.8, 90, drivetrain));
    scg.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  /* Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }*/
}
