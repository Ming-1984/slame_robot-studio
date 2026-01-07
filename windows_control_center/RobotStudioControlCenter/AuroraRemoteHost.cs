using System.ComponentModel;
using System.Diagnostics;
using System.Runtime.InteropServices;
using System.Text;

namespace RobotStudioControlCenter;

public sealed class AuroraRemoteHost : IDisposable
{
    private Process? process;
    private IntPtr mainWindowHandle = IntPtr.Zero;
    private IntPtr originalParent = IntPtr.Zero;
    private int? originalStyle;
    private Panel? hostPanel;
    private CancellationTokenSource? hideWindowsCts;
    private Task? hideWindowsTask;
    private IntPtr winEventHook = IntPtr.Zero;
    private Win32.WinEventDelegate? winEventProc;

    public bool IsRunning => process is not null && !process.HasExited;
    public int? ProcessId => process?.HasExited == false ? process.Id : null;
    public IntPtr MainWindowHandle => mainWindowHandle;

    public event Action<string>? Log;

    public async Task StartAsync(string exePath, CancellationToken cancellationToken)
    {
        if (IsRunning)
        {
            Log?.Invoke("测绘建图已在运行。");
            return;
        }

        StopHidingWindows();

        if (!File.Exists(exePath))
        {
            throw new FileNotFoundException("未找到测绘建图程序。", exePath);
        }

        var workingDirectory = Path.GetDirectoryName(exePath) ?? Environment.CurrentDirectory;

        // Start suspended + off-screen, install the "move off-screen" hooks, then resume.
        // This minimizes the chance of any flash before we can embed the window.
        var pi = default(Win32.PROCESS_INFORMATION);
        var si = new Win32.STARTUPINFO
        {
            cb = Marshal.SizeOf<Win32.STARTUPINFO>(),
            dwFlags = Win32.STARTF_USEPOSITION | Win32.STARTF_USESHOWWINDOW,
            wShowWindow = Win32.SW_SHOWNOACTIVATE,
            dwX = -32000,
            dwY = -32000,
        };

        var cmdLine = new StringBuilder($"\"{exePath}\"");
        if (!Win32.CreateProcess(
            exePath,
            cmdLine,
            IntPtr.Zero,
            IntPtr.Zero,
            false,
            Win32.CREATE_SUSPENDED,
            IntPtr.Zero,
            workingDirectory,
            ref si,
            out pi))
        {
            throw new Win32Exception(Marshal.GetLastWin32Error(), "启动测绘建图失败。");
        }

        try
        {
            StartHidingWindows((int)pi.dwProcessId, cancellationToken);

            var resume = Win32.ResumeThread(pi.hThread);
            if (resume == uint.MaxValue)
            {
                throw new Win32Exception(Marshal.GetLastWin32Error(), "启动测绘建图失败。");
            }
        }
        finally
        {
            _ = Win32.CloseHandle(pi.hThread);
            _ = Win32.CloseHandle(pi.hProcess);
        }

        process = await WaitForProcessAsync((int)pi.dwProcessId, cancellationToken);
        Log?.Invoke($"测绘建图已启动 (PID={process.Id})");

        mainWindowHandle = await WaitForMainWindowAsync(process, cancellationToken, hideImmediately: true);

        if (mainWindowHandle == IntPtr.Zero)
        {
            throw new InvalidOperationException("无法获取测绘建图主窗口句柄。");
        }

        Log?.Invoke($"测绘建图主窗口句柄: 0x{mainWindowHandle.ToInt64():X}");
    }

    public void EmbedInto(Panel panel)
    {
        if (mainWindowHandle == IntPtr.Zero || !Win32.IsWindow(mainWindowHandle))
        {
            throw new InvalidOperationException("测绘建图主窗口不可用，无法嵌入。");
        }

        hostPanel = panel;

        originalParent = Win32.GetParent(mainWindowHandle);
        originalStyle = Win32.GetWindowLong(mainWindowHandle, Win32.GWL_STYLE);

        var newStyle = originalStyle.Value;
        newStyle |= Win32.WS_CHILD;
        newStyle &= ~Win32.WS_CAPTION;
        newStyle &= ~Win32.WS_THICKFRAME;
        newStyle &= ~Win32.WS_MINIMIZEBOX;
        newStyle &= ~Win32.WS_MAXIMIZEBOX;
        newStyle &= ~Win32.WS_SYSMENU;

        Win32.SetParent(mainWindowHandle, panel.Handle);
        Win32.SetWindowLong(mainWindowHandle, Win32.GWL_STYLE, newStyle);

        StopHidingWindows();

        ResizeToHost();
    }

    public void ResizeToHost()
    {
        if (hostPanel is null || mainWindowHandle == IntPtr.Zero || !Win32.IsWindow(mainWindowHandle))
        {
            return;
        }

        Win32.SetWindowPos(
            mainWindowHandle,
            IntPtr.Zero,
            0,
            0,
            hostPanel.ClientSize.Width,
            hostPanel.ClientSize.Height,
            Win32.SWP_NOZORDER | Win32.SWP_NOACTIVATE | Win32.SWP_FRAMECHANGED | Win32.SWP_SHOWWINDOW);
    }

    public bool TryGetWindowRect(out Rectangle rect)
    {
        rect = default;
        if (!Win32.TryGetWindowRect(mainWindowHandle, out var r) || r is null)
        {
            return false;
        }

        rect = r.Value;
        return true;
    }

    public void Activate()
    {
        if (mainWindowHandle == IntPtr.Zero || !Win32.IsWindow(mainWindowHandle))
        {
            return;
        }

        _ = Win32.SetForegroundWindow(mainWindowHandle);
        _ = Win32.SetFocus(mainWindowHandle);
    }

    public void Stop()
    {
        if (process is null)
        {
            return;
        }

        StopHidingWindows();

        try
        {
            if (!process.HasExited)
            {
                Log?.Invoke("正在关闭测绘建图...");
                _ = process.CloseMainWindow();

                if (!process.WaitForExit(1500))
                {
                    process.Kill(entireProcessTree: true);
                }
            }
        }
        catch
        {
            // Ignore
        }
        finally
        {
            process.Dispose();
            process = null;
            mainWindowHandle = IntPtr.Zero;
            hostPanel = null;
            originalParent = IntPtr.Zero;
            originalStyle = null;
        }
    }

    public void Dispose()
    {
        Stop();
    }

    private static async Task<IntPtr> WaitForMainWindowAsync(Process p, CancellationToken ct, bool hideImmediately)
    {
        var timeoutAt = DateTime.UtcNow.AddSeconds(30);
        while (DateTime.UtcNow < timeoutAt && !ct.IsCancellationRequested)
        {
            try
            {
                p.Refresh();
                if (p.MainWindowHandle != IntPtr.Zero)
                {
                    var handle = p.MainWindowHandle;
                    if (hideImmediately)
                    {
                        MoveOffScreen(handle);
                    }

                    return handle;
                }
            }
            catch
            {
                return IntPtr.Zero;
            }

            await Task.Delay(10, ct);
        }

        return IntPtr.Zero;
    }

    private static async Task HideTopLevelWindowsAsync(int processId, CancellationToken ct)
    {
        while (!ct.IsCancellationRequested)
        {
            try
            {
                var windows = Win32.FindTopLevelWindowsByProcessId(processId);
                foreach (var hWnd in windows)
                {
                    if (hWnd == IntPtr.Zero || !Win32.IsWindow(hWnd))
                    {
                        continue;
                    }

                    if (Win32.IsWindowVisible(hWnd))
                    {
                        MoveOffScreen(hWnd);
                    }
                }
            }
            catch
            {
                // Ignore
            }

            await Task.Delay(20, ct);
        }
    }

    private static async Task<Process> WaitForProcessAsync(int pid, CancellationToken ct)
    {
        var timeoutAt = DateTime.UtcNow.AddSeconds(5);
        while (DateTime.UtcNow < timeoutAt && !ct.IsCancellationRequested)
        {
            try
            {
                return Process.GetProcessById(pid);
            }
            catch (ArgumentException)
            {
                // Not started yet.
            }

            await Task.Delay(10, ct);
        }

        return Process.GetProcessById(pid);
    }

    private void StartHidingWindows(int processId, CancellationToken ct)
    {
        hideWindowsCts = CancellationTokenSource.CreateLinkedTokenSource(ct);
        hideWindowsTask = HideTopLevelWindowsAsync(processId, hideWindowsCts.Token);

        winEventProc = (_, eventType, hwnd, idObject, _, _, _) =>
        {
            if (hwnd == IntPtr.Zero || idObject != Win32.OBJID_WINDOW)
            {
                return;
            }

            if (eventType != Win32.EVENT_OBJECT_CREATE && eventType != Win32.EVENT_OBJECT_SHOW)
            {
                return;
            }

            Win32.GetWindowThreadProcessId(hwnd, out var pid);
            if (pid != (uint)processId)
            {
                return;
            }

            var style = Win32.GetWindowLong(hwnd, Win32.GWL_STYLE);
            if ((style & Win32.WS_CHILD) != 0)
            {
                return;
            }

            MoveOffScreen(hwnd);
        };

        winEventHook = Win32.SetWinEventHook(
            Win32.EVENT_OBJECT_CREATE,
            Win32.EVENT_OBJECT_SHOW,
            IntPtr.Zero,
            winEventProc,
            (uint)processId,
            0,
            Win32.WINEVENT_OUTOFCONTEXT | Win32.WINEVENT_SKIPOWNPROCESS | Win32.WINEVENT_SKIPOWNTHREAD);
    }

    private void StopHidingWindows()
    {
        try
        {
            hideWindowsCts?.Cancel();
        }
        catch
        {
            // Ignore
        }

        hideWindowsCts?.Dispose();
        hideWindowsCts = null;
        hideWindowsTask = null;

        if (winEventHook != IntPtr.Zero)
        {
            try
            {
                _ = Win32.UnhookWinEvent(winEventHook);
            }
            catch
            {
                // Ignore
            }

            winEventHook = IntPtr.Zero;
        }

        winEventProc = null;
    }

    private static void MoveOffScreen(IntPtr hWnd)
    {
        if (hWnd == IntPtr.Zero || !Win32.IsWindow(hWnd))
        {
            return;
        }

        _ = Win32.SetWindowPos(
            hWnd,
            IntPtr.Zero,
            -32000,
            -32000,
            0,
            0,
            Win32.SWP_NOZORDER | Win32.SWP_NOACTIVATE | Win32.SWP_NOSIZE);
    }
}
