package frc.robot.subsystems.roller;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.AdvantageKitYAMS;
import frc.robot.util.SmartMotorIO;
import frc.robot.util.SmartMotorIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

/**
 * Example Roller subsystem using AdvantageKit + YAMS integration.
 *
 * <p>This demonstrates how to use the SmartMotorIO interface with YAMS configuration while
 * supporting AdvantageKit log replay.
 */
public class RollerAKit extends SubsystemBase {

  // The IO layer - abstracts hardware access for replay support
  private final SmartMotorIO io;

  // AutoLogged inputs - populated by IO layer, logged by AdvantageKit
  private final SmartMotorIOInputsAutoLogged inputs = new SmartMotorIOInputsAutoLogged();

  public RollerAKit() {
    // Create YAMS config (same as before, but without YAMS telemetry)
    SmartMotorControllerConfig config =
        new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.OPEN_LOOP)
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
            .withMotorInverted(true)
            .withIdleMode(MotorMode.BRAKE)
            .withStatorCurrentLimit(Amps.of(40));

    // Create the IO using the factory
    // This handles REAL/SIM/REPLAY modes automatically
    io =
        AdvantageKitYAMS.createTalonFX(
            config, DCMotor.getFalcon500(1), TelemetryVerbosity.HIGH, new TalonFX(14));
  }

  /**
   * Alternative constructor for dependency injection (useful for testing or custom IO).
   *
   * @param io The SmartMotorIO implementation to use
   */
  public RollerAKit(SmartMotorIO io) {
    this.io = io;
  }

  public AngularVelocity getVelocity() {
    // Read from logged inputs instead of directly from hardware
    return RadiansPerSecond.of(inputs.mechanismVelocityRadPerSec);
  }

  public void setDutyCycle(double dutyCycle) {
    io.setDutyCycle(dutyCycle);
  }

  public Command set(double dutyCycle) {
    return run(() -> setDutyCycle(dutyCycle))
        .finallyDo(interrupted -> setDutyCycle(0))
        .withName("SetRoller1");
  }

  @Override
  public void periodic() {
    // Update inputs from hardware (or replay log)
    io.updateInputs(inputs);

    // Send to AdvantageKit for logging/replay
    Logger.processInputs("Roller", inputs);

    Logger.recordOutput("Roller/VelocityRadPerSec", inputs.mechanismVelocityRadPerSec);
  }

  @Override
  public void simulationPeriodic() {
    // YAMS handles simulation physics
    io.simIterate();
  }
}
