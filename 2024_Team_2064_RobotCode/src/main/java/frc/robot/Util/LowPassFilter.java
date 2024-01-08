package frc.robot.Util;

public class LowPassFilter {
    private double alpha;
    private double prevValue;

    /**
     * Initializes a new LowPassFilter with the given alpha value
     * @param alpha The alpha value to use for the filter
     */
    public LowPassFilter(double alpha)
    {
        if (alpha <= 0 || alpha > 1){
            throw new IllegalArgumentException("Alpha must be between 0 and 1");
        }
        this.alpha = alpha;
        this.prevValue = 0;
    }


    /**
     * Filter The input data
     * @param input The input data to filter
     * @return The filtered data
     */
    public double filter(double input)
    {
        double output = alpha * input + (1 - alpha) * prevValue;
        prevValue = output;
        return output;
    }
    
}
