package frc.robot.util;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class EditableParameter {
    private GenericEntry entry = null;
    private double defaultValue;

    public EditableParameter(ShuffleboardTab tab, String label, double defaultValue, boolean enabled) {
        this.defaultValue = defaultValue;
        if (enabled)
            this.entry = tab.add(label, defaultValue).getEntry();
    }

    public EditableParameter(ShuffleboardTab tab, String label, double defaultValue) {
        this(tab, label, defaultValue, true);
    }

    public double getValue() {
        return entry != null ? entry.getDouble(defaultValue) : defaultValue;
    }
}