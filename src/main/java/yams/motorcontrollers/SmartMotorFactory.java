package yams.motorcontrollers;

import java.lang.reflect.Constructor;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

public class SmartMotorFactory {
  @FunctionalInterface
  public interface MotorControllerConstructor {
    SmartMotorController create(Object... params);
  }

  public static final Map<String, MotorControllerConstructor> availableControllers = new HashMap<>();

  static {
    register("com.ctre.phoenix6.hardware.TalonFX",
        "yams.motorcontrollers.remote.TalonFXWrapper");
    register("com.ctre.phoenix6.hardware.TalonFXS",
        "yams.motorcontrollers.remote.TalonFXSWrapper");
    register("com.revrobotics.spark.SparkBase",
        "yams.motorcontrollers.local.SparkWrapper");
    register("com.thethriftybot.ThriftyNova",
        "yams.motorcontrollers.local.NovaWrapper");
  }

  private static void register(String motorControllerClassName, String wrapperClassName) {
    try {
      Class<?> wrapperClass = Class.forName(wrapperClassName);

      MotorControllerConstructor constructor = (params) -> {
        try {
          for (Constructor<?> ctor : wrapperClass.getConstructors()) {
            if (ctor.getParameterCount() == params.length) {
              Class<?>[] paramTypes = ctor.getParameterTypes();
              boolean matches = true;
              for (int i = 0; i < paramTypes.length; i++) {
                if (!paramTypes[i].isAssignableFrom(params[i].getClass())) {
                  matches = false;
                  break;
                }
              }
              if (matches) {
                return (SmartMotorController) ctor.newInstance(params);
              }
            }
          }
          throw new RuntimeException("No matching constructor found in " + wrapperClassName);
        } catch (Exception e) {
          throw new RuntimeException(e);
        }
      };

      availableControllers.put(motorControllerClassName, constructor);

    } catch (ClassNotFoundException e) {
      // Wrapper class not found, ignore registration
    }
  }

  public static Optional<SmartMotorController> create(Object controllerInstance, Object... extraParams) {
    if (controllerInstance == null) {
      return Optional.empty();
    }

    Class<?> clazz = controllerInstance.getClass();

    while (clazz != null) {
      String className = clazz.getName();
      MotorControllerConstructor constructor = availableControllers.get(className);
      if (constructor != null) {
        return Optional.of(instantiateMotorController(constructor, controllerInstance, extraParams));
      }

      // Check interfaces implemented by this class
      for (Class<?> iface : clazz.getInterfaces()) {
        constructor = availableControllers.get(iface.getName());
        if (constructor != null) {
          return Optional.of(instantiateMotorController(constructor, controllerInstance, extraParams));
        }
      }

      clazz = clazz.getSuperclass();
    }

    return Optional.empty();
  }

  private static SmartMotorController instantiateMotorController(MotorControllerConstructor constructor,
      Object controllerInstance,
      Object[] extraParams) {
    Object[] params = new Object[extraParams.length + 1];
    params[0] = controllerInstance;
    System.arraycopy(extraParams, 0, params, 1, extraParams.length);
    return constructor.create(params);
  }
}
