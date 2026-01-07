using Microsoft.Web.WebView2.WinForms;

namespace RobotStudioControlCenter;

public class InputFriendlyWebView2 : WebView2
{
    protected override bool IsInputKey(Keys keyData)
    {
        var key = keyData & Keys.KeyCode;
        return key is Keys.Up or Keys.Down or Keys.Left or Keys.Right || base.IsInputKey(keyData);
    }
}

