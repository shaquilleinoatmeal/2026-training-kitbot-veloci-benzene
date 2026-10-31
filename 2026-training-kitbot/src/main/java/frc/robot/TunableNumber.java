package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TunableNumber {
  private final String key;
  private final double defaultValue;

  public TunableNumber(String name, double defaultValue) {
    this.key = "Tuning/" + name;
    this.defaultValue = defaultValue;
    SmartDashboard.setDefaultNumber(key, defaultValue);
  }

  public double get() {
    return SmartDashboard.getNumber(key, defaultValue);
  }
}
