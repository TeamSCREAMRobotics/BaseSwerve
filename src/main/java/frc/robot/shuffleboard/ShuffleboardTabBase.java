package frc.robot.shuffleboard;

import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public abstract class ShuffleboardTabBase {
    protected ShuffleboardTab mTab;

    public abstract void createEntries();

    protected GenericEntry createNumberEntry(String name, double defaultValue) {
        return mTab.add(name, defaultValue).withSize(1, 1).getEntry();
    }

    protected GenericEntry createBooleanEntry(String name, boolean defaultValue) {
        return mTab.add(name, defaultValue).withSize(1, 1).getEntry();
    }

    protected GenericEntry createStringEntry(String name, String defaultValue) {
        return mTab.add(name, defaultValue).withSize(1, 1).getEntry();
    }

    protected GenericEntry createEntry(String name, Object defaultValue) {
        return mTab.add(name, defaultValue).withSize(1, 1).getEntry();
    }

    protected GenericEntry createSlider(String name, double defaultValue, double minValue, double maxValue) {
        return mTab.add(name, defaultValue).withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", minValue, "max", maxValue)).getEntry();
    }

    protected GenericEntry createGraph(String name, double value) {
        return mTab.add(name, value).withWidget(BuiltInWidgets.kGraph).getEntry();
    }

    public abstract void periodic();

    protected double round(double number, int decimalPlaces) {
        double tmp = Math.pow(10, decimalPlaces);
        return Math.round(number * tmp) / tmp;
    }

    public ShuffleboardTab getTab() {
        return mTab;
    }
}