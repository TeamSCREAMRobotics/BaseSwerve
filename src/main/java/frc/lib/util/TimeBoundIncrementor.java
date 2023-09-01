package frc.lib.util;

/**
 * Increments a time value within specified lower and upper bounds.
 */
public class TimeBoundIncrementor {
    
    private double mTime;
    private final double mLowerBound;
    private final double mUpperBound;
    public TimeBoundIncrementor(double lowerBound, double upperBound){
        this(lowerBound, upperBound, 0.0);
    }

    public TimeBoundIncrementor(double lowerBound, double upperBound, double initialValue){
        mLowerBound = lowerBound;
        mUpperBound = upperBound;
        mTime = initialValue;
    }

    public void setTime(double time){
        mTime = time;
    }

    public void increment(double increment){
        mTime += increment;
        if(mTime > mUpperBound) mTime = mUpperBound;
        else if(mTime < mLowerBound) mTime = mLowerBound;
    }

    public double getTime(){
        return mTime;
    }

    public boolean atMinimum(){
        return mTime == mLowerBound;
    }

    public boolean atMaximum(){
        return mTime == mLowerBound;
    }
}