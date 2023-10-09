package frc.robot.shuffleboard;

import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

import java.util.Set;

/**
 * Base class for Shuffleboard tabs.
 */
public abstract class ShuffleboardTabBase {
    protected ShuffleboardTab m_tab;

    private final Set<String> m_validPropertyKeys = Set.of("min", "max", "block increment", "center", "show value", "visible time", "color when true", "color when false", "orientation", "number of tick marks", "show voltage and current values", "show text", "precision", "show tick marks", "range", "major tick spacing", "starting angle", "show tick mark ring", "number of wheels", "wheel diameter", "show velocity vectors", "show crosshair", "crosshair color", "show controls", "rotation");

    public abstract void createEntries();

    /**
     * Creates a number entry with the given name and default value.
     *
     * @param name The name of the entry.
     * @param defaultValue The default value of the entry.
     * @param entryProps The EntryProperties to use for the entry.
     * @return The created number entry.
     */
    protected GenericEntry createNumberEntry(String name, double defaultValue, EntryProperties entryProps) {
        return createEntry(name, defaultValue, entryProps);
    }

    /**
     * Creates a boolean entry with the given name and default value.
     *
     * @param name The name of the entry.
     * @param defaultValue The default value of the entry.
     * @param entryProps The EntryProperties to use for the entry.
     * @return The created boolean entry.
     */
    protected GenericEntry createBooleanEntry(String name, boolean defaultValue, EntryProperties entryProps) {
        return createEntry(name, defaultValue, entryProps);
    }

    /**
     * Creates a new string entry with the given name and default value.
     *
     * @param name The name of the entry.
     * @param defaultValue The default value of the entry.
     * @param entryProps The EntryProperties to use for the entry.
     * @return The created string entry.
     */
    protected GenericEntry createStringEntry(String name, String defaultValue, EntryProperties entryProps) {
        return createEntry(name, defaultValue, entryProps);
    }

    /**
     * Creates a new entry with the given name and default value.
     *
     * @param name The name of the entry.
     * @param defaultValue The default value of the entry.
     * @param entryProps The EntryProperties to use for the entry.
     * @return The created entry.
     */
    protected GenericEntry createEntry(String name, Object defaultValue, EntryProperties entryProps) {
        SimpleWidget entry = m_tab.add(name, defaultValue);
        
        if (entryProps.xPos() != null && entryProps.yPos() != null) {
            entry = entry
                .withPosition(entryProps.xPos().intValue(), entryProps.yPos().intValue());
        }

        if (entryProps.width() != null && entryProps.height() != null) {
            entry = entry
                .withSize(entryProps.width().intValue(), entryProps.height().intValue());
        }
        
        if (entryProps.widgetType() != null) {
            entry = entry.withWidget(entryProps.widgetType());
        }
        
        if (entryProps.propertyMap() != null) {
            for (String key : entryProps.propertyMap().keySet()) {
                if (!m_validPropertyKeys.contains(key)) {
                    System.out.println("Invalid property key " + key + " in Shuffleboard entry: " + name);
                }
            }
            entry = entry.withProperties(entryProps.propertyMap());
        }
        
        return entry.getEntry();
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
        return m_tab;
    }

    public record EntryProperties(Integer xPos, Integer yPos, Integer width, Integer height, WidgetType widgetType, Map<String, Object> propertyMap){
        public EntryProperties(Integer xPos, Integer yPos, Integer width, Integer height, WidgetType widgetType) {
            this(xPos, yPos, width, height, widgetType, null);
        }
        public EntryProperties(Integer xPos, Integer yPos, Integer width, Integer height) {
            this(xPos, yPos, width, height, null);
        }
        public EntryProperties(Integer xPos, Integer yPos) {
            this(xPos, yPos, 1, 1);
        }
        public EntryProperties() {
            this(null, null, null, null);
        }
    }
}