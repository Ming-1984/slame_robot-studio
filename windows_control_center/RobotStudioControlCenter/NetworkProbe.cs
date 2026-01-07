using System.Net.NetworkInformation;
using System.Net.Sockets;

namespace RobotStudioControlCenter;

public static class NetworkProbe
{
    public static async Task<bool> PingAsync(string host, int timeoutMs)
    {
        try
        {
            using var ping = new Ping();
            var reply = await ping.SendPingAsync(host, timeoutMs);
            return reply.Status == IPStatus.Success;
        }
        catch
        {
            return false;
        }
    }

    public static async Task<bool> TcpPortOpenAsync(string host, int port, int timeoutMs)
    {
        try
        {
            using var tcpClient = new TcpClient();
            var connectTask = tcpClient.ConnectAsync(host, port);
            var completed = await Task.WhenAny(connectTask, Task.Delay(timeoutMs));
            return completed == connectTask && tcpClient.Connected;
        }
        catch
        {
            return false;
        }
    }
}

