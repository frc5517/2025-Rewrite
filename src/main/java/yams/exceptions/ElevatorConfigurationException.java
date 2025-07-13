package yams.exceptions;

public class ElevatorConfigurationException extends RuntimeException
{

  public ElevatorConfigurationException(String message, String result, String remedyFunction)
  {
    super(message + "!\n" + result + "\nPlease use ElevatorConfig." + remedyFunction + " to fix this error.");
  }
}
