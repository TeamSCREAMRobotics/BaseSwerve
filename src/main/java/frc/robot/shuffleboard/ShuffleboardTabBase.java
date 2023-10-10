package frc.robot.shuffleboard;

import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

/**
 * Base class for Shuffleboard tabs.
 */
public abstract class ShuffleboardTabBase {
    protected ShuffleboardTab m_tab;

    public abstract void createEntries();

    private GenericEntry createEntry(String name, Object defaultValue, EntryProperties entryProps, Widget widget){
        SimpleWidget entry = m_tab.add(name, defaultValue);
        
        if (entryProps.xPos() != null && entryProps.yPos() != null) {
            entry = entry
                .withPosition(entryProps.xPos().intValue(), entryProps.yPos().intValue());
        }

        if (entryProps.width() != null && entryProps.height() != null) {
            entry = entry
                .withSize(entryProps.width().intValue(), entryProps.height().intValue());
        }

        if(widget.type() != null){
            entry = entry.withWidget(widget.type());
        }

        if(widget.propertyMap() != null){
            entry = entry.withProperties(widget.propertyMap());
        }
        
        return entry.getEntry();
    }

    protected GenericEntry createNumberEntry(String name, double defaultValue, EntryProperties entryProps){
        return createEntry(name, defaultValue, entryProps, new Widget());
    }

    protected GenericEntry createNumberEntry(String name, double defaultValue, EntryProperties entryProps, Widget widget){
        return createEntry(name, defaultValue, entryProps, widget);
    }

    protected GenericEntry createStringEntry(String name, String defaultValue, EntryProperties entryProps){
        return createEntry(name, defaultValue, entryProps, new Widget());
    }

    protected GenericEntry createStringEntry(String name, String defaultValue, EntryProperties entryProps, Widget widget){
        return createEntry(name, defaultValue, entryProps, widget);
    }

    protected GenericEntry createBooleanEntry(String name, boolean defaultValue, EntryProperties entryProps){
        return createEntry(name, defaultValue, entryProps, new Widget());
    }

    protected GenericEntry createBooleanEntry(String name, boolean defaultValue, EntryProperties entryProps, Widget widget){
        return createEntry(name, defaultValue, entryProps, widget);
    }

    protected ComplexWidget createSendableEntry(String name, Sendable sendable, EntryProperties entryProps, Map<String, Object> propertyMap){
        ComplexWidget entry =  m_tab.add(name, sendable);

        if (entryProps.xPos() != null && entryProps.yPos() != null) {
            entry = entry
                .withPosition(entryProps.xPos().intValue(), entryProps.yPos().intValue());
        }

        if (entryProps.width() != null && entryProps.height() != null) {
            entry = entry
                .withSize(entryProps.width().intValue(), entryProps.height().intValue());
        }

        if(propertyMap != null){
            entry = entry.withProperties(propertyMap);
        }

        return entry;
    }

    protected ComplexWidget createSendableEntry(String name, Sendable sendable, EntryProperties entryProps){
        return createSendableEntry(name, sendable, entryProps, null);
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

    public record EntryProperties(Integer xPos, Integer yPos, Integer width, Integer height){
        public EntryProperties(Integer xPos, Integer yPos) {
            this(xPos, yPos, null, null);
        }
        public EntryProperties() {
            this(null, null, null, null);
        }
    }

    public record Widget(WidgetType type, Map<String, Object> propertyMap){
        public Widget(WidgetType type) {
            this(type, null);
        }
        private Widget(){
            this(null, null);
        }
    }
}