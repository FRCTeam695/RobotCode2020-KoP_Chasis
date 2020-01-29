/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.Joystick;
/**
 * Controller button indicies:
 * A: 1
 * B: 2
 * X: 3
 * Y: 4
 * 
 * Left X-axis: 0
 * Left Y-axis: 1
 * 
 */
/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //***************************************************************************/
  //SUBSYSTEMS INITIALIZED & CONSTRUCTED BELOW:
  //***************************************************************************/
  private final Motors RobotDriveMotors = new Motors();
  private final IntakeMotor IntakeMechanism = new IntakeMotor(4);
  //private final IntakeMotor BallLift = new IntakeMotor(6);
  private final BallDetector Detector = new BallDetector(0);
  //private final ModelTurret Turret = new ModelTurret(2,3);
  //private final CompressorController Compressor = new CompressorController();
  //private final HatchGrabber HatchSolenoid = new HatchGrabber(0);
  //private final SetColor ColorSensorUsed = new SetColor();

  //***************************************************************************/
  //USERINPUT STUFF (CONTROLLERS, JOYSTICK BUTTONS) INIT & CONSTRUCTED BELOW:
  //***************************************************************************/
  private Joystick ControllerDrive = new Joystick(0);
  //private Joystick ControllerAuxiliary = new Joystick(1);
  //private final JoystickButton AButton = new JoystickButton(ControllerDrive,1);
  //private final JoystickButton XButton = new JoystickButton(ControllerDrive,3);
  //private final JoystickButton YButton = new JoystickButton(ControllerDrive,4);
  private final POVButton UpPOV = new POVButton(ControllerDrive,0);
  private final POVButton DownPOV = new POVButton(ControllerDrive,180);

  //***************************************************************************/
  //COMMANDS INIT & CONSTRUCTED BELOW:
  //***************************************************************************/
  //private final TankDrive ActivateTankDrive = new TankDrive(RobotDriveMotors,ControllerDrive,1,5);
  private final MattDrive ActivateMattDrive = new MattDrive(RobotDriveMotors,ControllerDrive,1,4);
  private final EnableConstantIntake ActivateIntake = new EnableConstantIntake(IntakeMechanism,0.25);
  //private final EnableConstantIntake ActivateLift = new EnableConstantIntake(BallLift,-Constants.INTAKE_POWER);
  private final FindBall ActivateBallFinder = new FindBall(Detector, IntakeMechanism);
  private final ParallelCommandGroup ContinuousTeleop = new ParallelCommandGroup(ActivateIntake,ActivateBallFinder,ActivateMattDrive);
  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    //enable intake
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //YButton.whenPressed(new InstantCommand(HatchSolenoid::toggleHatchState, HatchSolenoid));
    UpPOV.whenPressed(new InstantCommand(() -> {ActivateIntake.incrementIntakePower(0.25);}));
    DownPOV.whenPressed(new InstantCommand(() -> {ActivateIntake.incrementIntakePower(-0.25);}));

  }


  /**
   * Use this to pass the teleop command to the main {@link Robot} class.
   *
   * @return the command to run in teleop
   */
  public Command getTeleopCommand() {
    return ContinuousTeleop;
  }
}
