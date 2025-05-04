package yams.exceptions;

public class PivotConfigurationException extends RuntimeException
{

  public PivotConfigurationException(String message, String result, String remedyFunction)
  {
    super(message + "!\n" + result + "\nPlease use PivotConfig." + remedyFunction + " to fix this error.");
  }
}
