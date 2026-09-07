package frc.lib.sensors;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngularAcceleration;
import frc.lib.CAN;
import frc.lib.configs.appliers.Pigeon2ConfigApplier;
import frc.robot.RobotConstants;

/** Pigeon 2 gyroscope */
public class GyroscopePigeon2 implements Gyroscope {

  private Pigeon2 gyroscope;

  private final StatusSignal<Angle> roll, pitch, yaw;
  private final StatusSignal<AngularVelocity> rollVelocity, pitchVelocity, yawVelocity;
  private final MutAngularAcceleration rollAcceleration, pitchAcceleration, yawAcceleration;

  public GyroscopePigeon2(CAN gyroscopeCAN) {

    gyroscope = new Pigeon2(gyroscopeCAN.id(), gyroscopeCAN.bus());

    roll = gyroscope.getRoll();
    pitch = gyroscope.getPitch();
    yaw = gyroscope.getYaw();

    rollVelocity = gyroscope.getAngularVelocityXWorld();
    pitchVelocity = gyroscope.getAngularVelocityYWorld();
    yawVelocity = gyroscope.getAngularVelocityZWorld();

    rollAcceleration = RotationsPerSecondPerSecond.mutable(0);
    pitchAcceleration = RotationsPerSecondPerSecond.mutable(0);
    yawAcceleration = RotationsPerSecondPerSecond.mutable(0);
  }

  @Override
  public void configure() {
    BaseStatusSignal.setUpdateFrequencyForAll(100, yaw, yawVelocity);
    BaseStatusSignal.setUpdateFrequencyForAll(10, roll, pitch, rollVelocity, pitchVelocity);

    Pigeon2ConfigApplier.applyFactoryDefault(gyroscope);
  }

  @Override
  public void getUpdatedVals(GyroscopeValues values) {
    values.roll = Degrees.of(roll.getValueAsDouble());
    values.pitch = Degrees.of(pitch.getValueAsDouble());
    values.yaw = Degrees.of(yaw.getValueAsDouble());
    values.rollVelocity = DegreesPerSecond.of(rollVelocity.getValueAsDouble());
    values.pitchVelocity = DegreesPerSecond.of(pitchVelocity.getValueAsDouble());
    values.yawVelocity = DegreesPerSecond.of(yawVelocity.getValueAsDouble());
    values.rollAcceleration = rollAcceleration;
    values.pitchAcceleration = pitchAcceleration;
    values.yawAcceleration = yawAcceleration;
  }

  @Override
  public void setYaw(Angle newYaw) {
    gyroscope.setYaw(newYaw.in(Degrees));
  }

  @Override
  public void periodic() {
    AngularVelocity prevRollVelocity = rollVelocity.getValue();
    AngularVelocity prevPitchVelocity = pitchVelocity.getValue();
    AngularVelocity prevYawVelocity = yawVelocity.getValue();
    BaseStatusSignal.refreshAll(roll, pitch, yaw, rollVelocity, pitchVelocity, yawVelocity);

    rollAcceleration.mut_replace(rollVelocity.getValue().minus(prevRollVelocity).div(RobotConstants.PERIODIC_DURATION));
    pitchAcceleration.mut_replace(pitchVelocity.getValue().minus(prevPitchVelocity).div(RobotConstants.PERIODIC_DURATION));
    yawAcceleration.mut_replace(yawVelocity.getValue().minus(prevYawVelocity).div(RobotConstants.PERIODIC_DURATION));
  }
}
