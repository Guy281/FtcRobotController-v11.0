package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.opencv.core.Scalar;

/**
 * An {@link ColorRange represents a 3-channel minimum/maximum
 *                      range for a given color space}
 */
public class ColorRange
{
    protected final ColorSpace colorSpace;
    protected final Scalar min;
    protected final Scalar max;

    public static final ColorRange GREEN = new ColorRange(
            ColorSpace.YCrCb,
            new Scalar( 0,  255, 145),
            new Scalar(170, 255,  0)
    );

    public static final ColorRange PURPLE = new ColorRange(
            ColorSpace.YCrCb,
            new Scalar( 50,  0,  255),
            new Scalar(255,  0,  230)
    );

    public ColorRange(ColorSpace colorSpace, Scalar min, Scalar max)
    {
        this.colorSpace = colorSpace;
        this.min = min;
        this.max = max;
    }
}

