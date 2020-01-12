/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeMotor;

public class EnableJoystickIntakeControl extends CommandBase {
  private IntakeMotor IntakeMotorControlled;
  private Joystick ControllerBeingRead;
  private int powerAxisId;
  /**
   * Creates a new EnableJoystickInakeControl.
   */
  public EnableJoystickIntakeControl(IntakeMotor IntakeMotorControlled,Joystick ControllerBeingRead,int powerAxisId) {
    this.IntakeMotorControlled = IntakeMotorControlled;
    this.ControllerBeingRead = ControllerBeingRead;
    this.powerAxisId = powerAxisId;
    addRequirements(IntakeMotorControlled);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    IntakeMotorControlled.setPower(ControllerBeingRead.getRawAxis(powerAxisId));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
