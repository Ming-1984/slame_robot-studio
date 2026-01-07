using Microsoft.Web.WebView2.Core;
using Microsoft.Web.WebView2.WinForms;
using System.ComponentModel;
using System.Diagnostics;

namespace RobotStudioControlCenter;

public sealed class MainForm : Form
{
    private readonly SettingsStore settingsStore = new();
    private readonly AppSettings settings;
    private readonly AuroraRemoteHost auroraRemoteHost = new();
    private readonly MappingFocusMessageFilter mappingFocusMessageFilter;

    private readonly TabControl tabs = new() { Dock = DockStyle.Fill };
    private TabPage? robotTab;
    private TabPage? workspaceTab;
    private TabPage? mappingTab;
    private TabPage? deviceTab;
    private TabPage? diagTab;

    private readonly InputFriendlyWebView2 robotStudioWebView = new() { Dock = DockStyle.Fill };
    private readonly InputFriendlyWebView2 deviceMgmtWebView = new() { Dock = DockStyle.Fill };
    private readonly InputFriendlyWebView2 workspaceRobotStudioWebView = new() { Dock = DockStyle.Fill };

    private readonly Panel auroraHostPanel = new() { Dock = DockStyle.Fill, BackColor = Color.Black };
    private readonly Panel workspaceAuroraHostPanel = new() { Dock = DockStyle.Fill, BackColor = Color.Black, TabStop = true };
    private readonly SplitContainer workspaceSplit = new() { Dock = DockStyle.Fill, Orientation = Orientation.Vertical, SplitterWidth = 6 };
    private readonly TextBox logsTextBox = new()
    {
        Dock = DockStyle.Fill,
        Multiline = true,
        ReadOnly = true,
        ScrollBars = ScrollBars.Vertical,
        Font = new Font("Consolas", 10),
    };

    private readonly StatusStrip statusStrip = new();
    private readonly ToolStripStatusLabel statusLabel = new() { Text = "就绪" };

    private readonly System.Windows.Forms.Timer probeTimer = new() { Interval = 5000 };
    private readonly System.Windows.Forms.Timer resizeDebounceTimer = new() { Interval = 120 };
    private readonly System.Windows.Forms.Timer focusAssistTimer = new() { Interval = 20 };
    private bool lastLeftMouseDown;
    private CancellationTokenSource? auroraStartCts;

    public MainForm()
    {
        settings = settingsStore.Load();
        mappingFocusMessageFilter = new MappingFocusMessageFilter(this);
        Application.AddMessageFilter(mappingFocusMessageFilter);

        Text = "Robot Studio";
        Width = 1400;
        Height = 900;
        StartPosition = FormStartPosition.CenterScreen;

        FormClosing += OnFormClosing;
        Resize += (_, _) => DebounceResizeToHost();
        ResizeEnd += (_, _) => auroraRemoteHost.ResizeToHost();

        auroraRemoteHost.Log += AppendLog;

        BuildUi();
        _ = InitializeWebAsync();

        statusStrip.Items.Add(statusLabel);
        Controls.Add(statusStrip);
        Controls.Add(tabs);
        statusStrip.Dock = DockStyle.Bottom;

        tabs.SelectedIndexChanged += (_, _) => OnTabChanged();
        Shown += async (_, _) => await StartAndEmbedAuroraOnStartupAsync();

        probeTimer.Tick += async (_, _) => await ProbeOnceAsync();
        probeTimer.Start();

        focusAssistTimer.Tick += (_, _) => FocusAssistTick();
        focusAssistTimer.Start();

        resizeDebounceTimer.Tick += (_, _) =>
        {
            resizeDebounceTimer.Stop();
            auroraRemoteHost.ResizeToHost();
        };

        AppendLog("Robot Studio 已启动。");
    }

    private void BuildUi()
    {
        workspaceSplit.Panel1.Controls.Add(workspaceRobotStudioWebView);
        workspaceSplit.Panel2.Controls.Add(workspaceAuroraHostPanel);
        workspaceAuroraHostPanel.Resize += (_, _) => auroraRemoteHost.ResizeToHost();
        workspaceSplit.SplitterMoved += (_, _) => auroraRemoteHost.ResizeToHost();

        robotTab = new TabPage("Robot Studio");
        robotTab.Controls.Add(robotStudioWebView);

        workspaceTab = new TabPage("工作台（并排）");
        workspaceTab.Controls.Add(workspaceSplit);

        mappingTab = new TabPage("测绘建图");
        mappingTab.Controls.Add(BuildAuroraTab());

        deviceTab = new TabPage("设备管理（官方）");
        deviceTab.Controls.Add(deviceMgmtWebView);

        diagTab = new TabPage("运行日志");
        diagTab.Controls.Add(logsTextBox);

        tabs.TabPages.Add(robotTab);
        tabs.TabPages.Add(workspaceTab);
        tabs.TabPages.Add(mappingTab);
        tabs.TabPages.Add(deviceTab);
        tabs.TabPages.Add(diagTab);

        workspaceRobotStudioWebView.GotFocus += (_, _) => Activate();
        robotStudioWebView.GotFocus += (_, _) => Activate();
        deviceMgmtWebView.GotFocus += (_, _) => Activate();

        workspaceAuroraHostPanel.MouseDown += (_, _) => auroraRemoteHost.Activate();
        workspaceAuroraHostPanel.GotFocus += (_, _) => auroraRemoteHost.Activate();
    }

    private Control BuildAuroraTab()
    {
        auroraHostPanel.TabStop = true;
        auroraHostPanel.Resize += (_, _) => auroraRemoteHost.ResizeToHost();
        auroraHostPanel.MouseDown += (_, _) => auroraRemoteHost.Activate();
        auroraHostPanel.GotFocus += (_, _) => auroraRemoteHost.Activate();
        return auroraHostPanel;
    }

    private async Task InitializeWebAsync()
    {
        await InitializeWebViewAsync(robotStudioWebView, settings.RobotStudioUrl);
        await InitializeWebViewAsync(workspaceRobotStudioWebView, settings.RobotStudioUrl);
        await InitializeWebViewAsync(deviceMgmtWebView, settings.AuroraDeviceManagementUrl);
    }

    private async Task InitializeWebViewAsync(WebView2 webView, string url)
    {
        try
        {
            await webView.EnsureCoreWebView2Async();
            webView.CoreWebView2.Settings.AreDefaultContextMenusEnabled = true;
            webView.CoreWebView2.Settings.IsZoomControlEnabled = true;
            webView.Source = new Uri(url);
        }
        catch (Exception ex) when (ex.GetType().Name.Contains("RuntimeNotFound", StringComparison.OrdinalIgnoreCase))
        {
            AppendLog("未找到 WebView2 Runtime。");

            var installer = FindWebView2Installer();
            if (installer is null)
            {
                MessageBox.Show(
                    this,
                    "未找到 WebView2 Runtime。\n请安装 Microsoft Edge WebView2 Runtime 后再运行 Robot Studio。",
                    "缺少组件",
                    MessageBoxButtons.OK,
                    MessageBoxIcon.Error);
                return;
            }

            var result = MessageBox.Show(
                this,
                "未找到 WebView2 Runtime。\n是否现在安装（离线安装器）？",
                "缺少组件",
                MessageBoxButtons.YesNo,
                MessageBoxIcon.Warning);

            if (result != DialogResult.Yes)
            {
                return;
            }

            try
            {
                var p = Process.Start(new ProcessStartInfo
                {
                    FileName = installer,
                    UseShellExecute = true,
                });

                if (p is not null)
                {
                    await p.WaitForExitAsync();
                }

                AppendLog("WebView2 Runtime 安装完成，正在重试初始化...");
                await webView.EnsureCoreWebView2Async();
                webView.Source = new Uri(url);
            }
            catch (Exception installEx)
            {
                AppendLog($"WebView2 Runtime 安装/初始化失败: {installEx.Message}");
                MessageBox.Show(
                    this,
                    "WebView2 Runtime 安装失败或未完成。\n请安装完成后重新打开 Robot Studio。",
                    "安装失败",
                    MessageBoxButtons.OK,
                    MessageBoxIcon.Error);
            }
        }
        catch (Exception ex)
        {
            AppendLog($"WebView 初始化失败: {ex.Message}");
        }
    }

    private async Task StartAndEmbedAuroraOnStartupAsync()
    {
        auroraStartCts?.Cancel();
        auroraStartCts = new CancellationTokenSource();

        var wasTopMost = TopMost;
        TopMost = true;
        try
        {
            BringToFront();
            Activate();
        }
        catch
        {
            // Ignore
        }

        var exePath = settingsStore.ResolveBundledAuroraRemoteExePath();
        if (exePath is null)
        {
            AppendLog("未找到测绘建图组件。");
            MessageBox.Show(
                this,
                "未找到测绘建图组件。\n请确认安装包完整且未删除测绘建图文件夹。",
                "缺少文件",
                MessageBoxButtons.OK,
                MessageBoxIcon.Error);

            TopMost = wasTopMost;
            return;
        }

        try
        {
            await auroraRemoteHost.StartAsync(exePath, auroraStartCts.Token);

            auroraHostPanel.CreateControl();
            auroraRemoteHost.EmbedInto(auroraHostPanel);
            AppendLog("测绘建图已自动启动并嵌入。");
            DebounceResizeToHost();
            OnTabChanged();
        }
        catch (Win32Exception ex)
        {
            AppendLog($"启动测绘建图失败: {ex.Message}");
            TryOfferInstallVcRedist();
        }
        catch (Exception ex)
        {
            AppendLog($"启动/嵌入失败: {ex.Message}");
            MessageBox.Show(this, $"启动/嵌入测绘建图失败：\n{ex.Message}", "错误", MessageBoxButtons.OK, MessageBoxIcon.Error);
        }
        finally
        {
            TopMost = wasTopMost;
            try
            {
                Activate();
            }
            catch
            {
                // Ignore
            }
        }
    }

    private void OnTabChanged()
    {
        try
        {
            if (tabs.SelectedTab == mappingTab)
            {
                EnsureMappingEmbedded(auroraHostPanel);
                auroraRemoteHost.ResizeToHost();
                auroraRemoteHost.Activate();
                return;
            }

            if (tabs.SelectedTab == workspaceTab)
            {
                EnsureMappingEmbedded(workspaceAuroraHostPanel);
                auroraRemoteHost.ResizeToHost();
                workspaceRobotStudioWebView.Focus();
                return;
            }

            if (tabs.SelectedTab == robotTab)
            {
                robotStudioWebView.Focus();
                return;
            }

            if (tabs.SelectedTab == deviceTab)
            {
                deviceMgmtWebView.Focus();
            }
        }
        catch
        {
            // Ignore
        }
    }

    private void EnsureMappingEmbedded(Panel targetHost)
    {
        if (!auroraRemoteHost.IsRunning || auroraRemoteHost.MainWindowHandle == IntPtr.Zero)
        {
            return;
        }

        try
        {
            targetHost.CreateControl();
            auroraRemoteHost.EmbedInto(targetHost);
        }
        catch (Exception ex)
        {
            AppendLog($"切换测绘建图显示失败: {ex.Message}");
        }
    }

    private void DebounceResizeToHost()
    {
        resizeDebounceTimer.Stop();
        resizeDebounceTimer.Start();
    }

    private async Task ProbeOnceAsync()
    {
        try
        {
            var robotHost = new Uri(settings.RobotStudioUrl).Host;
            var auroraHost = settings.AuroraIp;

            var robotPing = await NetworkProbe.PingAsync(robotHost, 800);
            var robotWeb = await NetworkProbe.TcpPortOpenAsync(robotHost, 8080, 800);
            var robotApi = await NetworkProbe.TcpPortOpenAsync(robotHost, 8000, 800);
            var robotWs = await NetworkProbe.TcpPortOpenAsync(robotHost, 8001, 800);

            var auroraPing = await NetworkProbe.PingAsync(auroraHost, 800);
            var auroraWeb = await NetworkProbe.TcpPortOpenAsync(auroraHost, 80, 800);
            var aurora7447 = await NetworkProbe.TcpPortOpenAsync(auroraHost, 7447, 800);
            var aurora1445 = await NetworkProbe.TcpPortOpenAsync(auroraHost, 1445, 800);

            statusLabel.Text =
                $"Robot Studio ping:{Bool(robotPing)} web8080:{Bool(robotWeb)} api8000:{Bool(robotApi)} ws8001:{Bool(robotWs)} | " +
                $"雷达 ping:{Bool(auroraPing)} web80:{Bool(auroraWeb)} 7447:{Bool(aurora7447)} 1445:{Bool(aurora1445)}";
        }
        catch (Exception ex)
        {
            AppendLog($"自检失败: {ex.Message}");
        }
    }

    private static string Bool(bool v) => v ? "OK" : "FAIL";

    private void AppendLog(string message)
    {
        var line = $"[{DateTime.Now:HH:mm:ss}] {message}";
        if (InvokeRequired)
        {
            BeginInvoke(() => AppendLog(message));
            return;
        }

        logsTextBox.AppendText(line + Environment.NewLine);
        Debug.WriteLine(line);
    }

    private void OnFormClosing(object? sender, FormClosingEventArgs e)
    {
        Application.RemoveMessageFilter(mappingFocusMessageFilter);
        auroraStartCts?.Cancel();
        probeTimer.Stop();
        focusAssistTimer.Stop();
        resizeDebounceTimer.Stop();

        auroraRemoteHost.Stop();
    }

    private void FocusAssistTick()
    {
        try
        {
            if (tabs.SelectedTab != workspaceTab || !auroraRemoteHost.IsRunning)
            {
                lastLeftMouseDown = false;
                return;
            }

            if (!workspaceAuroraHostPanel.IsHandleCreated)
            {
                return;
            }

            var down = (Win32.GetAsyncKeyState(Win32.VK_LBUTTON) & 0x8000) != 0;
            if (down && !lastLeftMouseDown)
            {
                var p = Control.MousePosition;
                var rect = workspaceAuroraHostPanel.RectangleToScreen(workspaceAuroraHostPanel.ClientRectangle);
                if (rect.Contains(p))
                {
                    auroraRemoteHost.Activate();
                }
            }

            lastLeftMouseDown = down;
        }
        catch
        {
            // Ignore
        }
    }

    private void TryActivateMappingForClick()
    {
        if (!auroraRemoteHost.IsRunning)
        {
            return;
        }

        if (tabs.SelectedTab == mappingTab)
        {
            auroraRemoteHost.Activate();
            return;
        }

        if (tabs.SelectedTab == workspaceTab)
        {
            var p = Control.MousePosition;
            var rect = workspaceAuroraHostPanel.RectangleToScreen(workspaceAuroraHostPanel.ClientRectangle);
            if (rect.Contains(p))
            {
                auroraRemoteHost.Activate();
            }
        }
    }

    private sealed class MappingFocusMessageFilter : IMessageFilter
    {
        private const int WM_LBUTTONDOWN = 0x0201;
        private const int WM_LBUTTONDBLCLK = 0x0203;
        private const int WM_RBUTTONDOWN = 0x0204;
        private const int WM_MBUTTONDOWN = 0x0207;

        private readonly MainForm form;

        public MappingFocusMessageFilter(MainForm form)
        {
            this.form = form;
        }

        public bool PreFilterMessage(ref Message m)
        {
            if (m.Msg is WM_LBUTTONDOWN or WM_LBUTTONDBLCLK or WM_RBUTTONDOWN or WM_MBUTTONDOWN)
            {
                try
                {
                    form.TryActivateMappingForClick();
                }
                catch
                {
                    // Ignore
                }
            }

            return false;
        }
    }

    private void TryOfferInstallVcRedist()
    {
        try
        {
            var candidates = new[]
            {
                Path.Combine(AppContext.BaseDirectory, "installers", "VC_redist.x64.exe"),
                Path.Combine(AppContext.BaseDirectory, "aurora_app", "VC_redist.x64.exe"),
            };

            var installer = candidates.FirstOrDefault(File.Exists);
            if (installer is null)
            {
                MessageBox.Show(
                    this,
                    "启动测绘建图失败，可能缺少 VC++ 运行库。\n未找到离线安装器（VC_redist.x64.exe）。",
                    "依赖缺失",
                    MessageBoxButtons.OK,
                    MessageBoxIcon.Warning);
                return;
            }

            var result = MessageBox.Show(
                this,
                "启动测绘建图失败，可能缺少 VC++ 运行库。\n是否现在安装 VC_redist.x64.exe？",
                "依赖缺失",
                MessageBoxButtons.YesNo,
                MessageBoxIcon.Warning);

            if (result != DialogResult.Yes)
            {
                return;
            }

            _ = Process.Start(new ProcessStartInfo
            {
                FileName = installer,
                UseShellExecute = true,
            });
        }
        catch (Exception ex)
        {
            AppendLog($"引导安装 VC++ 运行库失败: {ex.Message}");
        }
    }

    private static string? FindWebView2Installer()
    {
        try
        {
            var candidates = new[]
            {
                Path.Combine(AppContext.BaseDirectory, "installers", "MicrosoftEdgeWebView2RuntimeInstallerX64.exe"),
                Path.Combine(AppContext.BaseDirectory, "installers", "MicrosoftEdgeWebView2RuntimeInstaller.exe"),
            };

            return candidates.FirstOrDefault(File.Exists);
        }
        catch
        {
            return null;
        }
    }
}
