package frc.robot;
 
import static frc.robot.Constants.OperatorConstants.*;
 
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.xrp.XRPOnBoardIO;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.AutonomousDistance;
import frc.robot.commands.AutonomousTime;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOXRP;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIO;
import frc.robot.subsystems.drive.DriveIOSim;
import frc.robot.subsystems.drive.DriveIOXRP;
 
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
 
  // Subsystems
  private final Drive drive;
  private final Arm m_arm;
  private final XRPOnBoardIO m_onboardIO = new XRPOnBoardIO();
 
  // Controller
  private final XboxController controller = new XboxController(kDriverControllerPort);
 
  // Autonomous chooser
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();
 
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Instantiate the correct DriveIO based on current mode.
    DriveIO driveIO =
        switch (Constants.currentMode) {
          case REAL -> new DriveIOXRP();
          case SIM -> new DriveIOSim();
          case REPLAY -> new DriveIO() {};
        };
    drive = new Drive(driveIO);
 
    ArmIO armIO =
        switch (Constants.currentMode) {
          case REAL -> new ArmIOXRP();
          case SIM -> new ArmIOSim();
          case REPLAY -> new ArmIO() {};
        };
    m_arm = new Arm(armIO);

    configureButtonBindings();
    configureAutonomous();
  }
 
  private void configureButtonBindings() {
    // Default command: arcade drive with left stick.
    // Axis 1 (left stick Y) is negated — joystick forward = negative value.
    drive.setDefaultCommand(
        new ArcadeDrive(
            drive,
            () -> -controller.getRawAxis(kForwardAxis),
            () -> -controller.getRawAxis(kRotationAxis)));
 
    // Onboard user button — prints to console.
    Trigger userButton = new Trigger(m_onboardIO::getUserButtonPressed);
    userButton
        .onTrue(new PrintCommand("USER Button Pressed"))
        .onFalse(new PrintCommand("USER Button Released"));
 
    // A button — arm low position.
    new JoystickButton(controller, XboxController.Button.kA.value)
        .onTrue(new InstantCommand(() -> m_arm.setAngle(ArmConstants.kLowAngleDeg), m_arm))
        .onFalse(new InstantCommand(() -> m_arm.stop(), m_arm));
        
    // B button — arm high position, stow on release.
    new JoystickButton(controller, XboxController.Button.kB.value)
        .onTrue(new InstantCommand(() -> m_arm.setAngle(ArmConstants.kHighAngleDeg), m_arm))
        .onFalse(new InstantCommand(() -> m_arm.stop(), m_arm));
  }
 
  private void configureAutonomous() {
    m_chooser.setDefaultOption("Auto Routine Distance", new AutonomousDistance(drive));
    m_chooser.addOption("Auto Routine Time", new AutonomousTime(drive));
    SmartDashboard.putData(m_chooser);
  }
 
  /**
   * Returns the autonomous command selected in SmartDashboard.
   *
   * @return the selected autonomous command
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}
