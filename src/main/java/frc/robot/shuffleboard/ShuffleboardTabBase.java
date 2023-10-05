package frc.robot.shuffleboard;

import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

import java.util.Set;

/**
 * Base class for Shuffleboard tabs.
 */
public abstract class ShuffleboardTabBase {
    protected ShuffleboardTab mTab;

    private Set<String> validPropertyKeys = Set.of("min", "max", "block increment", "center", "show value", "visible time", "color when true", "color when false", "orientation", "number of tick marks", "show voltage and current values", "show text", "precision", "show tick marks", "range", "major tick spacing", "starting angle", "show tick mark ring", "number of wheels", "wheel diameter", "show velocity vectors", "show crosshair", "crosshair color", "show controls", "rotation");

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
     * @param entryInfo The EntryInfo to use for the entry.
     * @return The created number entry.
     */
    protected GenericEntry createNumberEntry(String name, double defaultValue, EntryInfo entryInfo) {
        return mTab.add(name, defaultValue)
        .withPosition(entryInfo.getXPosition(), entryInfo.getYPosition())
        .withSize(entryInfo.getWidth(), entryInfo.getHeight())
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
     * @param entryInfo The EntryInfo to use for the entry.
     * @return The created boolean entry.
     */
    protected GenericEntry createBooleanEntry(String name, boolean defaultValue, EntryInfo entryInfo) {
        return mTab.add(name, defaultValue)
        .withPosition(entryInfo.getXPosition(), entryInfo.getYPosition())
        .withSize(entryInfo.getWidth(), entryInfo.getHeight())
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
     * @param entryInfo The EntryInfo to use for the entry.
     * @return The created string entry.
     */
    protected GenericEntry createStringEntry(String name, String defaultValue, EntryInfo entryInfo) {
        return mTab.add(name, defaultValue)
        .withPosition(entryInfo.getXPosition(), entryInfo.getYPosition())
        .withSize(entryInfo.getWidth(), entryInfo.getHeight())
        .getEntry();
    }

    /**
     * Creates a new entry with the given name and default value.
     *
     * @param name The name of the entry.
     * @param defaultValue The default value of the entry.
     * @return The created entry.
     */
    protected GenericEntry createEntry(String name, Object defaultValue, BuiltInWidgets widgetType, Map<String, Object> propertyMap, EntryInfo entryInfo) {
        if (propertyMap != null) {
            for (String key : propertyMap.keySet()) {
                if (!validPropertyKeys.contains(key)) {
                    throw new IllegalArgumentException("Invalid property key: " + key);
                }
            }
        }
        
        SimpleWidget entry = mTab.add(name, defaultValue);
        
        if (entryInfo != null) {
            entry = entry
                .withPosition(entryInfo.getXPosition(), entryInfo.getYPosition())
                .withSize(entryInfo.getWidth(), entryInfo.getHeight());
        }
        
        if (widgetType != null) {
            entry = entry.withWidget(widgetType);
        }
        
        if (propertyMap != null) {
            entry = entry.withProperties(propertyMap);
        }
        
        return entry.getEntry();
    }

    /**
     * Creates a new entry with the given name and default value.
     *
     * @param name The name of the entry.
     * @param defaultValue The default value of the entry.
     * @param widgetType 
     * @param entryInfo The EntryInfo to use for the entry.
     * @return The created entry.
     */
    protected GenericEntry createEntry(String name, Object defaultValue, BuiltInWidgets widgetType) {
        return createEntry(name, defaultValue, widgetType, null, null);
    }

    /**
     * Creates a new entry with the given name and default value.
     *
     * @param name The name of the entry.
     * @param defaultValue The default value of the entry.
     * @param widgetType The WidgetType to use for the entry.
     * @param entryInfo The EntryInfo to use for the entry.
     * @return The created entry.
     */
    protected GenericEntry createEntry(String name, Object defaultValue, BuiltInWidgets widgetType, EntryInfo entryInfo) {
        return createEntry(name, defaultValue, widgetType, null, entryInfo);
    }

    /**
     * Creates a new entry with the given name and default value.
     *
     * @param name The name of the entry.
     * @param defaultValue The default value of the entry.
     * @param widgetType The WidgetType to use for the entry.
     * @param entryInfo The EntryInfo to use for the entry.
     * @return The created entry.
     */
    protected GenericEntry createEntry(String name, Object defaultValue, BuiltInWidgets widgetType, Map<String, Object> propertyMap) {
        return createEntry(name, defaultValue, widgetType, propertyMap, null);
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

    public class EntryInfo {
        private final int xPos;
        private final int yPos;
        private final int width;
        private final int height;
    
        public EntryInfo(int xPos, int yPos, int width, int height) {
            this.xPos = xPos;
            this.yPos = yPos;
            this.width = width;
            this.height = height;
        }

        public EntryInfo(int xPos, int yPos) {
            this.xPos = xPos;
            this.yPos = yPos;
            this.width = 1;
            this.height = 1;
        }
    
        public int getXPosition() {
            return xPos;
        }

        public int getYPosition() {
            return yPos;
        }
    
        public int getWidth() {
            return width;
        }
    
        public int getHeight() {
            return height;
        }
    }
    
}