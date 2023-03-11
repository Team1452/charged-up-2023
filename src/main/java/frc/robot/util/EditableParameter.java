package frc.robot.util;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class EditableParameter {
    private GenericEntry entry;

    public EditableParameter(ShuffleboardTab tab, String label, double defaultValue) {
        this.entry = tab.add(label, defaultValue).getEntry();
    }

    public double getValue() {
        return entry.getDouble(0);
    }
}