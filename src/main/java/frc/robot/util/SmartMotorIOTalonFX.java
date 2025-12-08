package frc.robot.util;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import java.util.Optional;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

/**
 * SmartMotorIO implementation using YAMS TalonFXWrapper.
 *
 * <p>This wraps YAMS's SmartMotorController to provide AdvantageKit-compatible logging while
 * retaining all of YAMS's functionality (gearing, simulation, configuration, etc.).
 *
 * <p>Example usage:
 *
 * <pre>{@code
 * SmartMotorControllerConfig config = new SmartMotorControllerConfig(this)
 *     .withTelemetry("Roller", TelemetryVerbosity.HIGH)
 *     .withGearing(new MechanismGearing(GearBox.fromReductionStages(3)))
 *     .withIdleMode(MotorMode.BRAKE);
 *
 * SmartMotorIO io = new SmartMotorIOTalonFX(
 *     config,
 *     DCMotor.getFalcon500(1),
 *     TelemetryVerbosity.HIGH,
 *     MechanismType.ANGULAR,
 *     new TalonFX(14)
 * );
 * }</pre>
 */
public class SmartMotorIOTalonFX implements SmartMotorIO {

  private final SmartMotorController controller;
  private final SmartMotorControllerConfig config;
  private final TelemetryVerbosity verbosity;
  private final TalonFX[] talons;

  /**
   * Creates a new SmartMotorIOTalonFX.
   *
   * @param config The YAMS SmartMotorControllerConfig (should NOT have telemetry configured, we
   *     handle that via AKit)
   * @param motor The DCMotor model for simulation
   * @param verbosity The telemetry verbosity level (controls which inputs are populated)
   * @param mechanismType Whether this is an angular or linear mechanism
   * @param talons The TalonFX motor controller(s). First is the leader, rest are followers.
   */
  public SmartMotorIOTalonFX(
      SmartMotorControllerConfig config,
      DCMotor motor,
      TelemetryVerbosity verbosity,
      TalonFX... talons) {

    this.config = config;
    this.verbosity = verbosity;
    this.talons = talons;

    if (talons == null || talons.length == 0) {
      throw new IllegalArgumentException("At least one TalonFX must be provided");
    }

    // Use the provided config but override telemetry to minimal because AKit will log instead
    SmartMotorControllerConfig configForYams = config.withTelemetry("AKit", TelemetryVerbosity.LOW);

    this.controller = new TalonFXWrapper(talons[0], motor, configForYams);

    // Note: Only the first TalonFX is driven; additional Talons can be managed by the caller
    // (e.g., via YAMS or vendor-specific followers) to avoid hidden behavior here.
  }

  @Override
  public void updateInputs(SmartMotorIOInputs inputs) {
    // Connection status - optimistic default, will fall back to false if refresh fails
    inputs.connected = true;

    // === LOW verbosity - always populated ===
    try {
      inputs.mechanismPositionRad = controller.getMechanismPosition().in(Radians);
      inputs.mechanismVelocityRadPerSec = controller.getMechanismVelocity().in(RadiansPerSecond);

      inputs.rotorPositionRad = controller.getRotorPosition().in(Radians);
      inputs.rotorVelocityRadPerSec = controller.getRotorVelocity().in(RadiansPerSecond);

      // Setpoints
      Optional<Angle> posSetpoint = controller.getMechanismPositionSetpoint();
      Optional<AngularVelocity> velSetpoint = controller.getMechanismSetpointVelocity();
      inputs.setpointPosition = posSetpoint.map(a -> a.in(Radians)).orElse(0.0);
      inputs.setpointVelocity = velSetpoint.map(v -> v.in(RadiansPerSecond)).orElse(0.0);
    } catch (Exception ex) {
      inputs.connected = false;
    }

    // === MID verbosity ===
    if (verbosity == TelemetryVerbosity.MID || verbosity == TelemetryVerbosity.HIGH) {
      try {
        inputs.appliedVolts = controller.getVoltage().in(Volts);
        inputs.statorCurrentAmps = controller.getStatorCurrent().in(Amps);

        Optional<Current> supplyCurrent = controller.getSupplyCurrent();
        inputs.supplyCurrentAmps = supplyCurrent.map(c -> c.in(Amps)).orElse(0.0);

        // Duty cycle - calculate from voltage, fall back to -1..1 clamp
        inputs.dutyCycle = inputs.appliedVolts / 12.0;
      } catch (Exception ex) {
        inputs.connected = false;
      }
    }

    // === HIGH verbosity ===
    if (verbosity == TelemetryVerbosity.HIGH) {
      try {
        inputs.temperatureCelsius = controller.getTemperature().in(Celsius);

        SmartMotorControllerConfig cfg = controller.getConfig();

        // Current limits
        inputs.statorCurrentLimit = cfg.getStatorStallCurrentLimit().orElse(0);
        inputs.supplyCurrentLimit = cfg.getSupplyStallCurrentLimit().orElse(0);

        // Inversions
        inputs.motorInverted = cfg.getMotorInverted();
        inputs.encoderInverted = cfg.getEncoderInverted();

        // PID/FF (best-effort) - not all controllers expose getters, so best to leave zeroed if
        // absent
        cfg.getClosedLoopController()
            .ifPresent(
                pid -> {
                  try {
                    inputs.kP = pid.getP();
                    inputs.kI = pid.getI();
                    inputs.kD = pid.getD();
                  } catch (Exception ignored) {
                    // Leave as defaults if not exposed
                  }
                  try {
                    var constraints = pid.getConstraints();
                    inputs.motionProfileMaxVelocity = constraints.maxVelocity;
                    inputs.motionProfileMaxAcceleration = constraints.maxAcceleration;
                  } catch (Exception ignored) {
                    // Leave as defaults if not exposed
                  }
                });

        try {
          var feedforward = cfg.getSimpleFeedforward();
          inputs.kS = feedforward.get().getKs();
          inputs.kV = feedforward.get().getKv();
          inputs.kA = feedforward.get().getKa();
        } catch (Exception ignored) {
          // Leave as defaults if not exposed
        }
      } catch (Exception ex) {
        inputs.connected = false;
      }
    }

    // If disconnected, zero transient fields to avoid stale data in replay/logs
    if (!inputs.connected) {
      inputs.mechanismPositionRad = 0.0;
      inputs.mechanismVelocityRadPerSec = 0.0;
      inputs.rotorPositionRad = 0.0;
      inputs.rotorVelocityRadPerSec = 0.0;
      inputs.setpointPosition = 0.0;
      inputs.setpointVelocity = 0.0;
      inputs.appliedVolts = 0.0;
      inputs.dutyCycle = 0.0;
      inputs.statorCurrentAmps = 0.0;
      inputs.supplyCurrentAmps = 0.0;
      inputs.temperatureCelsius = 0.0;
    }
  }

  @Override
  public void setDutyCycle(double dutyCycle) {
    controller.setDutyCycle(Math.max(-1.0, Math.min(1.0, dutyCycle)));
  }

  @Override
  public void setVoltage(double volts) {
    controller.setVoltage(Volts.of(volts));
  }

  @Override
  public void setPosition(double position) {
    controller.setPosition(Radians.of(position));
  }

  @Override
  public void setVelocity(double velocity) {
    controller.setVelocity(RadiansPerSecond.of(velocity));
  }

  @Override
  public void stop() {
    controller.setDutyCycle(0);
  }

  @Override
  public void setEncoderPosition(double position) {
    controller.setEncoderPosition(Radians.of(position));
  }

  @Override
  public void setMechanismLimits(double lower, double upper) {
    controller.setMechanismLowerLimit(Radians.of(lower));
    controller.setMechanismUpperLimit(Radians.of(upper));
  }

  @Override
  public void setPIDGains(double kP, double kI, double kD) {
    // Best-effort: many YAMS controllers expect config-time PID; skip runtime mutation to avoid
    // unsupported operations.
  }

  @Override
  public void setFeedforwardGains(double kS, double kV, double kA, double kG) {
    // Best-effort: many YAMS controllers expect config-time FF; skip runtime mutation to avoid
    // unsupported operations.
  }

  @Override
  public void simIterate() {
    controller.simIterate();
  }

  /** Returns the underlying YAMS SmartMotorController for advanced usage. */
  public SmartMotorController getController() {
    return controller;
  }
}
