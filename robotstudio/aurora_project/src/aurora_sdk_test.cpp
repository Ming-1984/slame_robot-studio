#include <iostream>
#include <string>

// Include Aurora SDK headers
#include "aurora_pubsdk_inc.h"
#include "cxx/slamtec_remote_public.hxx"

using namespace std;
using namespace rp::standalone::aurora;

int main(int argc, char* argv[]) {
    cout << "Aurora SDK Compatibility Test" << endl;
    cout << "=============================" << endl;
    
    // Test 1: Get SDK version info
    cout << "Test 1: Getting SDK version information..." << endl;
    
    slamtec_aurora_sdk_version_info_t version_info;
    slamtec_aurora_sdk_errorcode_t result = slamtec_aurora_sdk_get_version_info(&version_info);
    
    if (result == SLAMTEC_AURORA_SDK_ERRORCODE_OK) {
        cout << "✅ SDK Version Test PASSED" << endl;
        cout << "   SDK Name: " << version_info.sdk_name << endl;
        cout << "   SDK Version: " << version_info.sdk_version_string << endl;
        cout << "   Build Time: " << version_info.sdk_build_time << endl;
        cout << "   Feature Flags: 0x" << hex << version_info.sdk_feature_flags << dec << endl;
    } else {
        cout << "❌ SDK Version Test FAILED (Error code: " << result << ")" << endl;
        return 1;
    }
    
    cout << endl;
    
    // Test 2: Create session (without connecting to device)
    cout << "Test 2: Creating SDK session..." << endl;

    slamtec_aurora_sdk_session_handle_t session_handle;
    slamtec_aurora_sdk_errorcode_t error_code;
    session_handle = slamtec_aurora_sdk_create_session(nullptr, 0, nullptr, &error_code);

    if (error_code == SLAMTEC_AURORA_SDK_ERRORCODE_OK && session_handle != nullptr) {
        cout << "✅ Session Creation Test PASSED" << endl;
        cout << "   Session Handle: " << session_handle << endl;

        // Clean up session
        slamtec_aurora_sdk_release_session(session_handle);
        cout << "   Session released successfully" << endl;
    } else {
        cout << "❌ Session Creation Test FAILED (Error code: " << error_code << ")" << endl;
        return 1;
    }
    
    cout << endl;
    
    // Test 3: Test C++ wrapper functionality
    cout << "Test 3: Testing C++ wrapper..." << endl;

    try {
        // Create a session using C++ wrapper
        slamtec_aurora_sdk_errorcode_t cpp_error_code;
        auto session = RemoteSDK::CreateSession(nullptr, SDKConfig(), &cpp_error_code);
        if (session && cpp_error_code == SLAMTEC_AURORA_SDK_ERRORCODE_OK) {
            cout << "✅ C++ Wrapper Test PASSED" << endl;
            cout << "   C++ session created successfully" << endl;

            // Clean up C++ session
            RemoteSDK::DestroySession(session);
            cout << "   C++ session destroyed successfully" << endl;
        } else {
            cout << "❌ C++ Wrapper Test FAILED - Could not create session (Error code: " << cpp_error_code << ")" << endl;
            return 1;
        }
    } catch (const exception& e) {
        cout << "❌ C++ Wrapper Test FAILED - Exception: " << e.what() << endl;
        return 1;
    } catch (...) {
        cout << "❌ C++ Wrapper Test FAILED - Unknown exception" << endl;
        return 1;
    }
    
    cout << endl;
    cout << "🎉 All Aurora SDK compatibility tests PASSED!" << endl;
    cout << "The Aurora SDK is compatible with this platform." << endl;
    
    return 0;
}
