using System.Text.Json;

namespace RobotStudioControlCenter;

public sealed class SettingsStore
{
    private static readonly JsonSerializerOptions JsonOptions = new()
    {
        PropertyNamingPolicy = JsonNamingPolicy.CamelCase,
        WriteIndented = true,
    };

    public static string SettingsDirectory =>
        Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData), "RobotStudio");

    public static string SettingsPath => Path.Combine(SettingsDirectory, "settings.json");

    public AppSettings Load()
    {
        try
        {
            if (!File.Exists(SettingsPath))
            {
                return new AppSettings();
            }

            var json = File.ReadAllText(SettingsPath);
            return JsonSerializer.Deserialize<AppSettings>(json, JsonOptions) ?? new AppSettings();
        }
        catch
        {
            return new AppSettings();
        }
    }

    public void Save(AppSettings settings)
    {
        Directory.CreateDirectory(SettingsDirectory);
        var json = JsonSerializer.Serialize(settings, JsonOptions);
        File.WriteAllText(SettingsPath, json);
    }

    public string? ResolveBundledAuroraRemoteExePath()
    {
        var primary = Path.Combine(AppContext.BaseDirectory, "aurora_app", "aurora_remote.exe");
        if (File.Exists(primary))
        {
            return primary;
        }

        return FindAuroraRemoteExePathNearBaseDirectory();
    }

    private static string? FindAuroraRemoteExePathNearBaseDirectory()
    {
        try
        {
            var current = new DirectoryInfo(Path.GetFullPath(AppContext.BaseDirectory));
            for (var depth = 0; depth < 10 && current is not null; depth++)
            {
                var candidates = new[]
                {
                    Path.Combine(current.FullName, "aurora_remote.exe"),
                    Path.Combine(current.FullName, "aurora_app", "aurora_remote.exe"),
                };

                var hit = candidates.FirstOrDefault(File.Exists);
                if (!string.IsNullOrWhiteSpace(hit))
                {
                    return hit;
                }

                current = current.Parent;
            }

            return null;
        }
        catch
        {
            return null;
        }
    }
}
