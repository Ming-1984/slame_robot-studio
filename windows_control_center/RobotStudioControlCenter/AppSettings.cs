namespace RobotStudioControlCenter;

public sealed class AppSettings
{
    public string RobotStudioUrl { get; set; } = "http://192.168.4.1:8080/";
    public string RobotApiBaseUrl { get; set; } = "http://192.168.4.1:8000";
    public string AuroraDeviceManagementUrl { get; set; } = "http://192.168.11.1/index.html";

    public string AuroraIp { get; set; } = "192.168.11.1";
}
