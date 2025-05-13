package frc.robot.utils;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.UsbPort;

public class ToolBox {
  // Utility method to adjust joystick values
  private DoubleSupplier adjustJoystick(DoubleSupplier input, boolean negate) {
      return () -> {
          double x = input.getAsDouble();
          if(negate) {
              x = -x;
          }
          x = MathUtil.applyDeadband(x, UsbPort.kDriveDeadband);
          x = Math.pow(Math.abs(x), 0.25) * Math.signum(x);
          System.out.print(x);
          return x;
      };
}

}
