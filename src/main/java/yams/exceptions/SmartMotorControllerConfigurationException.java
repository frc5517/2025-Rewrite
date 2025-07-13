package yams.exceptions;

public class SmartMotorControllerConfigurationException extends RuntimeException
{

  public SmartMotorControllerConfigurationException(String message, String result, String remedyFunction)
  {
    super(
        message + "!\n" + result + "\nPlease use SmartMotorControllerConfig." + remedyFunction + " to fix this error.");
  }
}
