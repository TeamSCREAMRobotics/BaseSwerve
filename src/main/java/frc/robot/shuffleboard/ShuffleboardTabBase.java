package frc.robot.shuffleboard;

import java.util.Map;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import java.util.Objects;

/**
 * Base class for Shuffleboard tabs.
 */
public abstract class ShuffleboardTabBase {
    protected ShuffleboardTab mTab;

    public abstract void createEntries();

    /**
     * Creates a number entry with the given name and default value.
     *
     * @param name The name of the entry.
     * @param defaultValue The default value of the entry.
     * @return The created number entry.
     */
    protected GenericEntry createNumberEntry(String name, double defaultValue) {
        return mTab.add(name, defaultValue).withSize(1, 1).getEntry();
    }

    /**
     * Creates a number entry with the given name and default value.
     *
     * @param name The name of the entry.
     * @param defaultValue The default value of the entry.
     * @return The created number entry.
     */
    protected GenericEntry createNumberEntry(String name, double defaultValue, CellInfo cellInfo) {
        return mTab.add(name, defaultValue)
        .withPosition(cellInfo.getColumnIndex(), cellInfo.getRowIndex())
        .withSize(cellInfo.getCellWidth(), cellInfo.getCellHeight())
        .getEntry();
    }

    /**
     * Creates a boolean entry with the given name and default value.
     *
     * @param name The name of the entry.
     * @param defaultValue The default value of the entry.
     * @return The created boolean entry.
     */
    protected GenericEntry createBooleanEntry(String name, boolean defaultValue) {
        return mTab.add(name, defaultValue).withSize(1, 1).getEntry();
    }

    /**
     * Creates a boolean entry with the given name and default value.
     *
     * @param name The name of the entry.
     * @param defaultValue The default value of the entry.
     * @return The created boolean entry.
     */
    protected GenericEntry createBooleanEntry(String name, boolean defaultValue, CellInfo cellInfo) {
        return mTab.add(name, defaultValue)
        .withPosition(cellInfo.getColumnIndex(), cellInfo.getRowIndex())
        .withSize(cellInfo.getCellWidth(), cellInfo.getCellHeight())
        .getEntry();
    }

    /**
     * Creates a new string entry with the given name and default value.
     *
     * @param name The name of the entry.
     * @param defaultValue The default value of the entry.
     * @return The created string entry.
     */
    protected GenericEntry createStringEntry(String name, String defaultValue) {
        return mTab.add(name, defaultValue).withSize(1, 1).getEntry();
    }

    /**
     * Creates a new string entry with the given name and default value.
     *
     * @param name The name of the entry.
     * @param defaultValue The default value of the entry.
     * @return The created string entry.
     */
    protected GenericEntry createStringEntry(String name, String defaultValue, CellInfo cellInfo) {
        return mTab.add(name, defaultValue)
        .withPosition(cellInfo.getColumnIndex(), cellInfo.getRowIndex())
        .withSize(cellInfo.getCellWidth(), cellInfo.getCellHeight())
        .getEntry();
    }

    /**
     * Creates a new entry with the given name and default value.
     *
     * @param name The name of the entry.
     * @param defaultValue The default value of the entry.
     * @return The created entry.
     */
    protected GenericEntry createEntry(String name, Object defaultValue) {
        return mTab.add(name, defaultValue).withSize(1, 1).getEntry();
    }

    /**
     * Creates a new entry with the given name and default value.
     *
     * @param name The name of the entry.
     * @param defaultValue The default value of the entry.
     * @return The created entry.
     */
    protected GenericEntry createEntry(String name, Object defaultValue, CellInfo cellInfo) {
        return mTab.add(name, defaultValue)
        .withPosition(cellInfo.getColumnIndex(), cellInfo.getRowIndex())
        .withSize(cellInfo.getCellWidth(), cellInfo.getCellHeight())
        .getEntry();
    }

    /**
     * Creates a slider widget entry with the given name, default value, minimum value, and maximum value.
     *
     * @param name The name of the entry.
     * @param defaultValue The default value of the entry.
     * @param minValue The minimum value of the slider.
     * @param maxValue The maximum value of the slider.
     * @return The created entry.
     */
    protected GenericEntry createSlider(String name, double defaultValue, double minValue, double maxValue) {
        return mTab.add(name, defaultValue)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", minValue, "max", maxValue))
        .withSize(3, 1)
        .getEntry();
    }

    /**
     * Creates a slider widget entry with the given name, default value, minimum value, and maximum value.
     *
     * @param name The name of the entry.
     * @param defaultValue The default value of the entry.
     * @param minValue The minimum value of the slider.
     * @param maxValue The maximum value of the slider.
     * @return The created entry.
     */
    protected GenericEntry createSlider(String name, double defaultValue, double minValue, double maxValue, CellInfo cellInfo) {
        return mTab.add(name, defaultValue)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", minValue, "max", maxValue))
        .withPosition(cellInfo.getColumnIndex(), cellInfo.getRowIndex())
        .withSize(cellInfo.getCellWidth(), cellInfo.getCellHeight())
        .getEntry();
    }

    /**
     * Creates a graph widget entry with the given name and default value
     *
     * @param name The name of the entry.
     * @param defaultValue The default value of the entry.
     * @return The created entry.
     */
    protected GenericEntry createGraph(String name, double defaultValue) {
        return mTab.add(name, defaultValue)
        .withWidget(BuiltInWidgets.kGraph)
        .getEntry();
    }

    /**
     * Creates a graph widget entry with the given name and default value
     *
     * @param name The name of the entry.
     * @param defaultValue The default value of the entry.
     * @return The created entry.
     */
    protected GenericEntry createGraph(String name, double defaultValue, CellInfo cellInfo) {
        return mTab.add(name, defaultValue)
        .withWidget(BuiltInWidgets.kGraph)
        .withPosition(cellInfo.getColumnIndex(), cellInfo.getRowIndex())
        .withSize(cellInfo.getCellWidth(), cellInfo.getCellHeight())
        .getEntry();
    }

    public abstract void periodic();

    /**
     * Rounds a given number to the specified number of decimal places.
     *
     * @param number The number to be rounded.
     * @param decimalPlaces The number of decimal places to round to.
     * @return The rounded number.
     */
    protected double round(double number, int decimalPlaces) {
        double tmp = Math.pow(10, decimalPlaces);
        return Math.round(number * tmp) / tmp;
    }

    public ShuffleboardTab getTab() {
        return mTab;
    }

    public class CellInfo {
        private final int columnIndex;
        private final int rowIndex;
        private final int cellWidth;
        private final int cellHeight;
    
        public CellInfo(int columnIndex, int rowIndex, int cellWidth, int cellHeight) {
            this.columnIndex = rowIndex;
            this.rowIndex = columnIndex;
            this.cellWidth = cellWidth;
            this.cellHeight = cellHeight;
        }
    
        public int getColumnIndex() {
            return columnIndex;
        }
    
        public int getRowIndex() {
            return rowIndex;
        }
    
        public int getCellWidth() {
            return cellWidth;
        }
    
        public int getCellHeight() {
            return cellHeight;
        }
    }
    
}