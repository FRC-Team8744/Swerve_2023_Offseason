// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Drivetrain;

public class AutonomousCommand extends CommandBase {
  private final Drivetrain m_drivetrain;
  /** Creates a new AutonomousCommand. */
  public AutonomousCommand(Drivetrain subsystem) {
    m_drivetrain = subsystem;
        addRequirements(m_drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrain.Drive(0.0, 0.0);//throttle, steering
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.Drive(0.0, 0.0);
    m_drivetrain.BrakeNow();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_DistanceInches < 0) {
      if (m_drivetrain.checkdistanceInches() <= m_startpoint + m_DistanceInches) return true;
  } else {
      if (m_drivetrain.checkdistanceInches() >= m_startpoint + m_DistanceInches) return true;

  }
    return false;
  }
}
