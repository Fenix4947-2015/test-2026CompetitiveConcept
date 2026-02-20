package frc.util;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;

public class DriveInputSmoother {
    private static final double kJoystickDeadband = 0.15;
    private static final double kCurveExponent = 1.5;
    private static final double kTranslationSlewRate = 3.0; // units per second
    private static final double kRotationSlewRate = 3.0;    // units per second

    private final DoubleSupplier forwardInput;
    private final DoubleSupplier leftInput;
    private final DoubleSupplier rotationInput;

    private final SlewRateLimiter forwardLimiter = new SlewRateLimiter(kTranslationSlewRate);
    private final SlewRateLimiter leftLimiter = new SlewRateLimiter(kTranslationSlewRate);
    private final SlewRateLimiter rotationLimiter = new SlewRateLimiter(kRotationSlewRate);

    public DriveInputSmoother(DoubleSupplier forwardInput, DoubleSupplier leftInput, DoubleSupplier rotationInput) {
        this.forwardInput = forwardInput;
        this.leftInput = leftInput;
        this.rotationInput = rotationInput;
    }

    public DriveInputSmoother(DoubleSupplier forwardInput, DoubleSupplier leftInput) {
        this(forwardInput, leftInput, () -> 0);
    }

    public ManualDriveInput getSmoothedInput() { 
        final double rawForward = forwardInput.getAsDouble();
        final double rawLeft = leftInput.getAsDouble();
        final double rawRotation = rotationInput.getAsDouble();

        final double deadbandedForward = MathUtil.applyDeadband(rawForward, kJoystickDeadband);
        final double deadbandedLeft = MathUtil.applyDeadband(rawLeft, kJoystickDeadband);
        final double deadbandedRotation = MathUtil.applyDeadband(rawRotation, kJoystickDeadband);

        final double curvedForward = MathUtil.copyDirectionPow(deadbandedForward, kCurveExponent);
        final double curvedLeft = MathUtil.copyDirectionPow(deadbandedLeft, kCurveExponent);
        final double curvedRotation = MathUtil.copyDirectionPow(deadbandedRotation, kCurveExponent);

        final double limitedForward = forwardLimiter.calculate(curvedForward);
        final double limitedLeft = leftLimiter.calculate(curvedLeft);
        final double limitedRotation = rotationLimiter.calculate(curvedRotation);

        return new ManualDriveInput(
            limitedForward, 
            limitedLeft, 
            limitedRotation
        );
    }
}
