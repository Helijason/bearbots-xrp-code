package frc.robot.subsystems.arm;
 
public class ArmIOSim implements ArmIO {
  private double commandedAngleDeg = ArmConstants.kStowedAngleDeg;
 
  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.commandedAngleDeg = commandedAngleDeg;
  }

  @Override
  public void setAngle(double angleDeg) {
      commandedAngleDeg = angleDeg;
  }
}
 