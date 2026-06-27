package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  static class ArmIOInputs {
    public double commandedAngleDeg = ArmConstants.kStowedAngleDeg;
  }

  default void updateInputs(ArmIOInputs inputs) {}

  default void setAngle(double angleDeg) {}
}
