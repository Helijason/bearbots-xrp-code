// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.scoop;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class Scoop extends SubsystemBase {
    public enum Goal {
        FLAT,
        CARRY,
        DUMP
    }

    private final ScoopIO io;
    private final ScoopIOInputsAutoLogged inputs = new ScoopIOInputsAutoLogged();
    private Goal goal = Goal.FLAT;

    // Mechanism2d: vertical column on the front of the robot
    private final LoggedMechanism2d mechanism = new LoggedMechanism2d(2, 2);
    private final LoggedMechanismRoot2d root = mechanism.getRoot("ScoopPivot", 1.0, 0.1);
    private final LoggedMechanismLigament2d scoopLigament =
        root.append(new LoggedMechanismLigament2d("Scoop", 0.15, 90));

  public Scoop(ScoopIO io) {
    this.io = io;
  }

  public void setGoal(Goal newGoal) {
    goal = newGoal;
  }

  public Command setGoalCommand(Goal newGoal) {
    return runOnce(() -> setGoal(newGoal));
  }

  public void stop() {
    setGoal(Goal.FLAT);
  }

  @Override
  public void periodic() {
      io.updateInputs(inputs);
      Logger.processInputs("Scoop", inputs);

      double targetAngle = switch (goal) {
          case FLAT  -> ScoopConstants.kFlatAngleDeg;
          case CARRY -> ScoopConstants.kCarryAngleDeg;
          case DUMP  -> ScoopConstants.kDumpAngleDeg;
      };

      double clamped = MathUtil.clamp(
          targetAngle,
          ScoopConstants.kMinAngleDeg,
          ScoopConstants.kMaxAngleDeg);

      io.setAngle(clamped);
      Logger.recordOutput("Scoop/Goal", goal.toString());

      // 90° = FLAT (up), 45° = CARRY, 0° = DUMP (forward)
      scoopLigament.setAngle(90 - inputs.commandedAngleDeg);
      Logger.recordOutput("Scoop/Mechanism2d", mechanism);
  }

   /** Expose Mechanism2d for aggregation in Robot.java. */
  public LoggedMechanism2d getMechanism() {
      return mechanism;
  }
}