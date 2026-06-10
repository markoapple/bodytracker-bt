#include "ui/desktop_ui.h"

#include "core/logging.h"

#ifdef _WIN32

#include <windows.h>
#include <windowsx.h>
#include <objbase.h>
#include <WebView2.h>
#include <wrl.h>
#include <wrl/event.h>

#include <nlohmann/json.hpp>

#include <filesystem>
#include <algorithm>
#include <string>

namespace bt {
namespace {

using Microsoft::WRL::Callback;
using Microsoft::WRL::ComPtr;

constexpr UINT_PTR kRefreshTimer = 1001;
constexpr wchar_t kUiHost[] = L"bodytracker.local";
constexpr wchar_t kUiUrl[] = L"https://bodytracker.local/index.html";

std::wstring ToWide(const std::string& value) {
    if (value.empty()) {
        return {};
    }
    const int size = MultiByteToWideChar(CP_UTF8, 0, value.data(), static_cast<int>(value.size()), nullptr, 0);
    std::wstring out(static_cast<std::size_t>(size), L'\0');
    MultiByteToWideChar(CP_UTF8, 0, value.data(), static_cast<int>(value.size()), out.data(), size);
    return out;
}

std::string ToUtf8(const wchar_t* value) {
    if (!value) {
        return {};
    }
    const int size = WideCharToMultiByte(CP_UTF8, 0, value, -1, nullptr, 0, nullptr, nullptr);
    if (size <= 1) {
        return {};
    }
    std::string out(static_cast<std::size_t>(size - 1), '\0');
    WideCharToMultiByte(CP_UTF8, 0, value, -1, out.data(), size, nullptr, nullptr);
    return out;
}

std::wstring JsonToWide(const nlohmann::json& json) {
    return ToWide(json.dump());
}

std::string LastWin32Error(const char* action) {
    const DWORD code = GetLastError();
    LPWSTR buffer = nullptr;
    FormatMessageW(
        FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
        nullptr,
        code,
        MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
        reinterpret_cast<LPWSTR>(&buffer),
        0,
        nullptr);
    std::string message = std::string(action) + " failed";
    if (code != 0) {
        message += " (Win32 " + std::to_string(code) + ")";
    }
    if (buffer) {
        message += ": " + ToUtf8(buffer);
        LocalFree(buffer);
    }
    return message;
}

std::filesystem::path ExecutableDirectory() {
    std::wstring buffer(MAX_PATH, L'\0');
    DWORD size = GetModuleFileNameW(nullptr, buffer.data(), static_cast<DWORD>(buffer.size()));
    while (size == buffer.size()) {
        buffer.resize(buffer.size() * 2);
        size = GetModuleFileNameW(nullptr, buffer.data(), static_cast<DWORD>(buffer.size()));
    }
    buffer.resize(size);
    const std::filesystem::path exe_path(buffer);
    return exe_path.has_parent_path() ? exe_path.parent_path() : std::filesystem::current_path();
}

std::filesystem::path UiFolder() {
    return ExecutableDirectory() / "ui";
}

bool JsonBool(const nlohmann::json& json, std::initializer_list<const char*> path, bool fallback = false) {
    const nlohmann::json* cursor = &json;
    for (const char* key : path) {
        if (!cursor->is_object() || !cursor->contains(key)) {
            return fallback;
        }
        cursor = &(*cursor)[key];
    }
    return cursor->is_boolean() ? cursor->get<bool>() : fallback;
}

double JsonNumber(const nlohmann::json& json, std::initializer_list<const char*> path, double fallback = 0.0) {
    const nlohmann::json* cursor = &json;
    for (const char* key : path) {
        if (!cursor->is_object() || !cursor->contains(key)) {
            return fallback;
        }
        cursor = &(*cursor)[key];
    }
    return cursor->is_number() ? cursor->get<double>() : fallback;
}

std::string JsonString(const nlohmann::json& json, std::initializer_list<const char*> path, const std::string& fallback = {}) {
    const nlohmann::json* cursor = &json;
    for (const char* key : path) {
        if (!cursor->is_object() || !cursor->contains(key)) {
            return fallback;
        }
        cursor = &(*cursor)[key];
    }
    return cursor->is_string() ? cursor->get<std::string>() : fallback;
}

std::string UiFolderError() {
    const auto folder = UiFolder();
    const auto index = folder / "index.html";
    if (!std::filesystem::exists(index)) {
        return "Desktop UI files are missing. Expected " + index.string();
    }
    return {};
}

nlohmann::json ParseWebMessage(ICoreWebView2WebMessageReceivedEventArgs* args) {
    LPWSTR raw = nullptr;
    std::string text;
    if (SUCCEEDED(args->TryGetWebMessageAsString(&raw)) && raw) {
        text = ToUtf8(raw);
        CoTaskMemFree(raw);
    } else if (SUCCEEDED(args->get_WebMessageAsJson(&raw)) && raw) {
        text = ToUtf8(raw);
        CoTaskMemFree(raw);
    }
    if (text.empty()) {
        return nlohmann::json::object();
    }
    auto parsed = nlohmann::json::parse(text);
    if (parsed.is_string()) {
        return nlohmann::json::parse(parsed.get<std::string>());
    }
    return parsed;
}

class DesktopPetWindow {
public:
    bool Create(HINSTANCE instance, HWND main_window) {
        main_window_ = main_window;
        WNDCLASSW wc{};
        wc.lpfnWndProc = &DesktopPetWindow::WndProc;
        wc.hInstance = instance;
        wc.lpszClassName = L"BodytrackerDesktopPet";
        wc.hCursor = LoadCursor(nullptr, IDC_HAND);
        wc.hbrBackground = nullptr;
        if (!RegisterClassW(&wc) && GetLastError() != ERROR_CLASS_ALREADY_EXISTS) {
            return false;
        }

        const int width = 286;
        const int height = 154;
        RECT work_area{0, 0, GetSystemMetrics(SM_CXSCREEN), GetSystemMetrics(SM_CYSCREEN)};
        SystemParametersInfoW(SPI_GETWORKAREA, 0, &work_area, 0);
        const int x = std::max(16, static_cast<int>(work_area.right) - width - 24);
        const int y = std::max(16, static_cast<int>(work_area.bottom) - height - 24);
        hwnd_ = CreateWindowExW(
            WS_EX_TOPMOST | WS_EX_TOOLWINDOW | WS_EX_LAYERED,
            wc.lpszClassName,
            L"bodytracker pet",
            WS_POPUP,
            x,
            y,
            width,
            height,
            nullptr,
            nullptr,
            instance,
            this);
        if (!hwnd_) {
            return false;
        }
        SetLayeredWindowAttributes(hwnd_, RGB(32, 32, 32), 0, LWA_COLORKEY);
        ShowWindow(hwnd_, SW_SHOWNOACTIVATE);
        return true;
    }

    void Destroy() {
        if (hwnd_) {
            DestroyWindow(hwnd_);
            hwnd_ = nullptr;
        }
    }

    void UpdateState(nlohmann::json state) {
        state_ = std::move(state);
        ++state_version_;
        if (hwnd_) {
            SetWindowPos(hwnd_, HWND_TOPMOST, 0, 0, 0, 0, SWP_NOMOVE | SWP_NOSIZE | SWP_NOACTIVATE);
            InvalidateRect(hwnd_, nullptr, FALSE);
        }
    }

private:
    static LRESULT CALLBACK WndProc(HWND hwnd, UINT message, WPARAM wparam, LPARAM lparam) {
        DesktopPetWindow* self = nullptr;
        if (message == WM_NCCREATE) {
            const auto* create = reinterpret_cast<CREATESTRUCTW*>(lparam);
            self = reinterpret_cast<DesktopPetWindow*>(create->lpCreateParams);
            self->hwnd_ = hwnd;
            SetWindowLongPtrW(hwnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(self));
        } else {
            self = reinterpret_cast<DesktopPetWindow*>(GetWindowLongPtrW(hwnd, GWLP_USERDATA));
        }
        return self ? self->HandleMessage(message, wparam, lparam) : DefWindowProcW(hwnd, message, wparam, lparam);
    }

    LRESULT HandleMessage(UINT message, WPARAM wparam, LPARAM lparam) {
        switch (message) {
        case WM_LBUTTONDOWN:
            dragging_ = true;
            drag_start_ = {GET_X_LPARAM(lparam), GET_Y_LPARAM(lparam)};
            SetCapture(hwnd_);
            return 0;
        case WM_MOUSEMOVE:
            if (dragging_) {
                POINT screen{GET_X_LPARAM(lparam), GET_Y_LPARAM(lparam)};
                ClientToScreen(hwnd_, &screen);
                SetWindowPos(hwnd_, HWND_TOPMOST, screen.x - drag_start_.x, screen.y - drag_start_.y, 0, 0, SWP_NOSIZE | SWP_NOACTIVATE);
            }
            return 0;
        case WM_LBUTTONUP:
            dragging_ = false;
            ReleaseCapture();
            return 0;
        case WM_LBUTTONDBLCLK:
            if (main_window_) {
                ShowWindow(main_window_, SW_RESTORE);
                SetForegroundWindow(main_window_);
            }
            return 0;
        case WM_PAINT:
            Paint();
            return 0;
        default:
            return DefWindowProcW(hwnd_, message, wparam, lparam);
        }
    }

    void Fill(HDC dc, int x, int y, int w, int h, COLORREF color) {
        RECT r{x, y, x + w, y + h};
        HBRUSH brush = CreateSolidBrush(color);
        FillRect(dc, &r, brush);
        DeleteObject(brush);
    }

    void Pixel(HDC dc, int x, int y, int w, int h, COLORREF color, int scale = 4) {
        Fill(dc, x * scale, y * scale, w * scale, h * scale, color);
    }

    std::pair<std::wstring, std::wstring> Lines() const {
        const bool model_ok = JsonBool(state_, {"model", "exists"});
        const bool running = JsonBool(state_, {"runtime", "running"});
        const int open_cameras = static_cast<int>(JsonNumber(state_, {"cameras", "open_count"}));
        const std::string mode = JsonString(state_, {"config", "tracking", "mode"}, "stereo");
        const int need = mode == "monocular" ? 1 : 2;
        const bool calibrated = mode == "monocular" || JsonBool(state_, {"calibration", "tracking_ready"});
        const bool osc_blocked = JsonBool(state_, {"tracker_space", "osc_blocked"});
        const double total_ms = JsonNumber(state_, {"debug", "total_ms"}, JsonNumber(state_, {"debug", "pipeline_ms"}, -1.0));
        const double sent = JsonNumber(state_, {"debug", "osc", "sent_message_count"}, 0.0);
        const double left_lock = JsonNumber(state_, {"debug", "body_state", "diagnostics", "left_contact_lock_strength"}, -1.0);
        const double right_lock = JsonNumber(state_, {"debug", "body_state", "diagnostics", "right_contact_lock_strength"}, -1.0);
        const bool identity_uncertain = JsonBool(state_, {"debug", "body_state", "diagnostics", "left_right_identity_uncertain"});
        const bool occlusion = JsonBool(state_, {"debug", "body_state", "diagnostics", "occlusion_prediction_active"});
        const std::string degradation = JsonString(state_, {"debug", "degradation_mode"});
        const std::string floor_status = JsonString(state_, {"floor_assist", "status"});

        if (!model_ok) return {L"MODEL HUNT", L"put ONNX in models, then rescan"};
        if (open_cameras < need) return {L"CAMERA SNIFF", ToWide(std::to_string(open_cameras) + "/" + std::to_string(need) + " signals open")};
        if (!calibrated) return {L"CALIB QUEST", L"stereo needs calibration"};
        if (!running) return {L"READY NAP", L"setup good, start runtime"};
        if (osc_blocked) return {L"OSC GUARD", L"blocked until tracker-space is trusted"};
        if (identity_uncertain) return {L"LEFT/RIGHT CHECK", L"hold still, let identity settle"};
        if (occlusion) return {L"OCCLUSION SAVE", L"prediction active, move slower"};
        if (total_ms > 33.0) return {L"FRAME SPIKE", ToWide(std::to_string(static_cast<int>(total_ms)) + " ms, tracking may lag")};
        if (!degradation.empty()) return {L"TRACK WATCH", ToWide(degradation)};
        if (left_lock >= 0.0 || right_lock >= 0.0) {
            const int left_pct = static_cast<int>(std::clamp(left_lock, 0.0, 1.0) * 100.0);
            const int right_pct = static_cast<int>(std::clamp(right_lock, 0.0, 1.0) * 100.0);
            return {L"FOOT LOCKS", ToWide("L " + std::to_string(left_pct) + "% / R " + std::to_string(right_pct) + "%")};
        }
        if (!floor_status.empty() && floor_status != "disabled") return {L"FLOOR BUDDY", ToWide(floor_status)};
        return {L"OSC SPARKS", ToWide(std::to_string(static_cast<int>(sent)) + " messages sent")};
    }

    void Paint() {
        PAINTSTRUCT ps{};
        HDC dc = BeginPaint(hwnd_, &ps);
        RECT bounds{};
        GetClientRect(hwnd_, &bounds);
        Fill(dc, 0, 0, bounds.right, bounds.bottom, RGB(32, 32, 32));

        SetBkMode(dc, TRANSPARENT);
        const auto [title, body] = Lines();
        Fill(dc, 86, 10, 190, 72, RGB(255, 254, 250));
        RECT bubble_frame{86, 10, 276, 82};
        FrameRect(dc, &bubble_frame, reinterpret_cast<HBRUSH>(GetStockObject(BLACK_BRUSH)));
        HFONT font = CreateFontW(15, 0, 0, 0, FW_BLACK, FALSE, FALSE, FALSE, DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS, NONANTIALIASED_QUALITY, DEFAULT_PITCH, L"Arial");
        HFONT old_font = reinterpret_cast<HFONT>(SelectObject(dc, font));
        SetTextColor(dc, RGB(4, 4, 4));
        RECT title_rect{98, 20, 266, 40};
        DrawTextW(dc, title.c_str(), -1, &title_rect, DT_LEFT | DT_SINGLELINE | DT_END_ELLIPSIS);
        SetTextColor(dc, RGB(92, 86, 78));
        RECT body_rect{98, 42, 266, 72};
        DrawTextW(dc, body.c_str(), -1, &body_rect, DT_LEFT | DT_WORDBREAK | DT_END_ELLIPSIS);

        const int bounce = JsonBool(state_, {"runtime", "running"}) ? static_cast<int>(state_version_ % 2) : 0;
        SetViewportOrgEx(dc, 10, 28 - bounce, nullptr);
        DrawSprite(dc);
        SetViewportOrgEx(dc, 0, 0, nullptr);
        SelectObject(dc, old_font);
        DeleteObject(font);
        EndPaint(hwnd_, &ps);
    }

    void DrawSprite(HDC dc) {
        const COLORREF ink = RGB(68, 38, 48);
        const COLORREF skin = RGB(255, 199, 166);
        const COLORREF hair = RGB(255, 143, 163);
        const COLORREF hair_hi = RGB(255, 207, 181);
        const COLORREF sweater = RGB(80, 112, 76);
        const COLORREF skirt = RGB(92, 58, 66);
        const COLORREF blush = RGB(255, 93, 132);

        Pixel(dc, 2, 26, 15, 2, RGB(180, 162, 150));
        Pixel(dc, 1, 6, 3, 8, skin);
        Pixel(dc, 2, 5, 2, 5, hair_hi);
        Pixel(dc, 14, 5, 3, 9, skin);
        Pixel(dc, 15, 4, 2, 6, hair_hi);
        Pixel(dc, 0, 12, 6, 2, ink);
        Pixel(dc, 13, 12, 6, 2, ink);
        Pixel(dc, 4, 8, 11, 10, hair);
        Pixel(dc, 5, 9, 9, 8, skin);
        Pixel(dc, 3, 9, 2, 8, hair);
        Pixel(dc, 13, 9, 2, 8, hair);
        Pixel(dc, 6, 10, 2, 2, ink);
        Pixel(dc, 11, 10, 2, 2, ink);
        if (JsonBool(state_, {"debug", "body_state", "diagnostics", "occlusion_prediction_active"})) {
            Pixel(dc, 6, 11, 2, 1, ink);
            Pixel(dc, 11, 11, 2, 1, ink);
        }
        Pixel(dc, 5, 13, 2, 1, blush);
        Pixel(dc, 12, 13, 2, 1, blush);
        Pixel(dc, 9, 14, 1, 1, ink);
        Pixel(dc, 6, 7, 3, 3, hair_hi);
        Pixel(dc, 10, 7, 3, 4, hair_hi);
        const int tail_wag = JsonBool(state_, {"runtime", "running"}) ? static_cast<int>(state_version_ % 2) : 0;
        Pixel(dc, 15, 11 + tail_wag, 5, 4, skin);
        Pixel(dc, 16, 12 + tail_wag, 4, 2, hair_hi);
        Pixel(dc, 14, 9, 2, 2, blush);
        Pixel(dc, 17, 8, 1, 1, blush);
        Pixel(dc, 17, 11, 1, 1, blush);
        Pixel(dc, 7, 18, 6, 6, sweater);
        Pixel(dc, 5, 18, 2, 4, sweater);
        Pixel(dc, 13, 18, 2, 4, sweater);
        Pixel(dc, 6, 22, 8, 2, skirt);
        Pixel(dc, 7, 24, 2, 3, skin);
        Pixel(dc, 11, 24, 2, 3, skin);
        Pixel(dc, 6, 27, 3, 1, ink);
        Pixel(dc, 11, 27, 3, 1, ink);
    }

    HWND hwnd_ = nullptr;
    HWND main_window_ = nullptr;
    nlohmann::json state_;
    bool dragging_ = false;
    POINT drag_start_{};
    unsigned int state_version_ = 0;
};

class DesktopWindow {
public:
    explicit DesktopWindow(DesktopUiController& c) : controller(c) {}

    Status Run(const std::string& title) {
        const std::string ui_error = UiFolderError();
        if (!ui_error.empty()) {
            return Status::Error(StatusCode::FailedPrecondition, ui_error);
        }

        const HRESULT co = CoInitializeEx(nullptr, COINIT_APARTMENTTHREADED);
        co_initialized_ = SUCCEEDED(co);
        if (FAILED(co) && co != RPC_E_CHANGED_MODE) {
            return Status::Error(StatusCode::InternalError, "Failed to initialize COM for WebView2");
        }

        const HINSTANCE instance = GetModuleHandleW(nullptr);
        WNDCLASSW wc{};
        wc.lpfnWndProc = &DesktopWindow::WndProc;
        wc.hInstance = instance;
        wc.lpszClassName = L"BodytrackerDesktopUi";
        wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
        if (!RegisterClassW(&wc) && GetLastError() != ERROR_CLASS_ALREADY_EXISTS) {
            return Status::Error(StatusCode::InternalError, LastWin32Error("RegisterClassW"));
        }

        hwnd_ = CreateWindowExW(
            0,
            wc.lpszClassName,
            ToWide(title).c_str(),
            WS_OVERLAPPEDWINDOW,
            CW_USEDEFAULT,
            CW_USEDEFAULT,
            1240,
            860,
            nullptr,
            nullptr,
            instance,
            this);
        if (!hwnd_) {
            return Status::Error(StatusCode::InternalError, LastWin32Error("CreateWindowExW"));
        }

        ShowWindow(hwnd_, SW_SHOW);
        UpdateWindow(hwnd_);

        const HRESULT hr = CreateCoreWebView2EnvironmentWithOptions(
            nullptr,
            nullptr,
            nullptr,
            Callback<ICoreWebView2CreateCoreWebView2EnvironmentCompletedHandler>(
                [this](HRESULT result, ICoreWebView2Environment* environment) -> HRESULT {
                    if (FAILED(result) || !environment) {
                        MessageBoxW(hwnd_, L"WebView2 runtime is missing or failed to start. Install Microsoft Edge WebView2 Runtime.", L"bodytracker", MB_ICONERROR | MB_OK);
                        PostQuitMessage(1);
                        return S_OK;
                    }
                    environment->CreateCoreWebView2Controller(
                        hwnd_,
                        Callback<ICoreWebView2CreateCoreWebView2ControllerCompletedHandler>(
                            [this](HRESULT controller_result, ICoreWebView2Controller* webview_controller) -> HRESULT {
                                if (FAILED(controller_result) || !webview_controller) {
                                    MessageBoxW(hwnd_, L"Failed to create WebView2 controller.", L"bodytracker", MB_ICONERROR | MB_OK);
                                    PostQuitMessage(1);
                                    return S_OK;
                                }
                                webview_controller_ = webview_controller;
                                webview_controller_->get_CoreWebView2(&webview_);
                                Resize();
                                InitializeWebView();
                                return S_OK;
                            }).Get());
                    return S_OK;
                }).Get());
        if (FAILED(hr)) {
            return Status::Error(StatusCode::InternalError, "WebView2 runtime is missing or failed to initialize");
        }

        MSG msg{};
        while (GetMessageW(&msg, nullptr, 0, 0) > 0) {
            TranslateMessage(&msg);
            DispatchMessageW(&msg);
        }

        controller.OnUiClosed();
        if (co_initialized_) {
            CoUninitialize();
        }
        return Status::OK();
    }

private:
    static LRESULT CALLBACK WndProc(HWND hwnd, UINT message, WPARAM wparam, LPARAM lparam) {
        DesktopWindow* self = nullptr;
        if (message == WM_NCCREATE) {
            const auto* create = reinterpret_cast<CREATESTRUCTW*>(lparam);
            self = reinterpret_cast<DesktopWindow*>(create->lpCreateParams);
            self->hwnd_ = hwnd;
            SetWindowLongPtrW(hwnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(self));
        } else {
            self = reinterpret_cast<DesktopWindow*>(GetWindowLongPtrW(hwnd, GWLP_USERDATA));
        }

        if (self) {
            return self->HandleWindowMessage(message, wparam, lparam);
        }
        return DefWindowProcW(hwnd, message, wparam, lparam);
    }

    LRESULT HandleWindowMessage(UINT message, WPARAM wparam, LPARAM lparam) {
        switch (message) {
        case WM_SIZE:
            Resize();
            return 0;
        case WM_TIMER:
            if (wparam == kRefreshTimer) {
                PostState();
            }
            return 0;
        case WM_DESTROY:
            KillTimer(hwnd_, kRefreshTimer);
            pet_.Destroy();
            PostQuitMessage(0);
            return 0;
        default:
            return DefWindowProcW(hwnd_, message, wparam, lparam);
        }
    }

    void InitializeWebView() {
        if (!webview_) {
            return;
        }

        ComPtr<ICoreWebView2Settings> settings;
        if (SUCCEEDED(webview_->get_Settings(&settings)) && settings) {
#ifdef _DEBUG
            settings->put_AreDevToolsEnabled(TRUE);
#else
            settings->put_AreDevToolsEnabled(FALSE);
#endif
            settings->put_IsStatusBarEnabled(FALSE);
        }

        ComPtr<ICoreWebView2_3> webview3;
        if (FAILED(webview_->QueryInterface(IID_PPV_ARGS(&webview3))) || !webview3) {
            MessageBoxW(hwnd_, L"This WebView2 runtime is too old for local app files. Install the current Microsoft Edge WebView2 Runtime.", L"bodytracker", MB_ICONERROR | MB_OK);
            PostQuitMessage(1);
            return;
        }

        const std::wstring folder = UiFolder().wstring();
        const HRESULT map_result = webview3->SetVirtualHostNameToFolderMapping(
            kUiHost,
            folder.c_str(),
            COREWEBVIEW2_HOST_RESOURCE_ACCESS_KIND_DENY_CORS);
        if (FAILED(map_result)) {
            MessageBoxW(hwnd_, L"Failed to map desktop UI folder into WebView2.", L"bodytracker", MB_ICONERROR | MB_OK);
            PostQuitMessage(1);
            return;
        }

        webview_->add_WebMessageReceived(
            Callback<ICoreWebView2WebMessageReceivedEventHandler>(
                [this](ICoreWebView2*, ICoreWebView2WebMessageReceivedEventArgs* args) -> HRESULT {
                    HandleMessage(args);
                    return S_OK;
                }).Get(),
            nullptr);
        webview_->Navigate(kUiUrl);
        SetTimer(hwnd_, kRefreshTimer, 66, nullptr);
    }

    void Resize() {
        if (!webview_controller_ || !hwnd_) {
            return;
        }
        RECT bounds{};
        GetClientRect(hwnd_, &bounds);
        webview_controller_->put_Bounds(bounds);
    }

    void PostState() {
        if (!webview_) {
            return;
        }
        nlohmann::json state = controller.GetStateJson();
        nlohmann::json msg{{"type", "state"}, {"state", std::move(state)}};
        const auto wide = JsonToWide(msg);
        webview_->PostWebMessageAsJson(wide.c_str());
    }

    void HandleMessage(ICoreWebView2WebMessageReceivedEventArgs* args) {
        nlohmann::json reply;
        int id = 0;
        try {
            const auto request = ParseWebMessage(args);
            id = request.value("id", 0);
            const std::string command = request.value("command", "");
            const auto payload = request.value("payload", nlohmann::json::object());
            Logger::Instance().Write(LogLevel::Info, "ui_command received id=" + std::to_string(id) + " command=" + command);
            const auto result = controller.HandleCommand(command, payload);
            const bool result_ok = !result.is_object() || result.value("ok", true);
            Logger::Instance().Write(
                result_ok ? LogLevel::Info : LogLevel::Warn,
                "ui_command replied id=" + std::to_string(id) + " command=" + command +
                    " ok=" + std::string(result_ok ? "true" : "false") +
                    " status=" + result.value("status", std::string{}));
            reply = {
                {"type", "reply"},
                {"id", id},
                {"ok", true},
                {"result", result}
            };
        } catch (const std::exception& e) {
            reply = {{"type", "reply"}, {"id", id}, {"ok", false}, {"error", e.what()}};
        }
        const auto wide = JsonToWide(reply);
        if (webview_) {
            webview_->PostWebMessageAsJson(wide.c_str());
            PostState();
        }
    }

    DesktopUiController& controller;
    HWND hwnd_ = nullptr;
    bool co_initialized_ = false;
    DesktopPetWindow pet_;
    ComPtr<ICoreWebView2Controller> webview_controller_;
    ComPtr<ICoreWebView2> webview_;
};

} // namespace

Status RunDesktopUi(DesktopUiController& controller, const std::string& title) {
    DesktopWindow window(controller);
    return window.Run(title);
}

} // namespace bt

#else

namespace bt {

Status RunDesktopUi(DesktopUiController&, const std::string&) {
    return Status::Error(StatusCode::Unsupported, "Desktop UI is only available on Windows");
}

} // namespace bt

#endif
