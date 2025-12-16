#include <GL/gl.h>
#include <GLFW/glfw3.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <iostream>
#include <mutex>
#include <thread>
#include <vector>

struct InputState {
    // WASD 轮询（每帧更新）
    bool w = false, a = false, s = false, d = false;

    // 鼠标相对移动（每帧）
    double dx = 0.0;
    double dy = 0.0;

    // 简单甩头检测：单帧移动超过阈值
    bool flickLeft = false, flickRight = false, flickUp = false,
         flickDown = false;
};

class GlfwRgbViewer {
   public:
    using FrameCallback = std::function<void(const uint8_t* rgb, int w, int h)>;
    using InputCallback = std::function<void(const InputState&)>;

    GlfwRgbViewer() = default;
    ~GlfwRgbViewer() { shutdown(); }

  void makeContextCurrent() {
    if (window_) {
      glfwMakeContextCurrent(window_);
    }
  }

  // 让当前线程释放这个窗口的上下文，便于交给渲染线程
  void detachContext() { glfwMakeContextCurrent(nullptr); }

  bool init(int winW, int winH, const char* title) {
    if (!glfwInit()) {
      std::cerr << "glfwInit failed\n";
      return false;
        }

        // 用兼容性更好的 OpenGL 2.1 固定管线（不需要 glad/glew）
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);

        window_ = glfwCreateWindow(winW, winH, title, nullptr, nullptr);
        if (!window_) {
            std::cerr << "glfwCreateWindow failed\n";
            glfwTerminate();
            return false;
        }

        glfwMakeContextCurrent(window_);
        glfwSwapInterval(1);  // vsync, 想更低延迟可改 0

        // 让按键不会因为丢帧就丢事件：建议用轮询 + Sticky
        glfwSetInputMode(window_, GLFW_STICKY_KEYS, GLFW_TRUE);

        // 鼠标：锁定到窗口中心 + 隐藏指针（FPS 视角常用）
        glfwSetInputMode(window_, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

        // Raw mouse motion（如果支持，甩头更“原始”更好）
        if (glfwRawMouseMotionSupported()) {
            glfwSetInputMode(window_, GLFW_RAW_MOUSE_MOTION, GLFW_TRUE);
        }

        glfwSetWindowUserPointer(window_, this);
        glfwSetCursorPosCallback(window_, &GlfwRgbViewer::cursorPosCB);

        // OpenGL 基本状态
        glDisable(GL_DEPTH_TEST);
        glEnable(GL_TEXTURE_2D);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);  // RGB 3字节对齐问题

        return true;
    }

  void setInputCallback(InputCallback cb) { input_cb_ = std::move(cb); }
  bool isClosed() const { return closed_.load(); }

    // 你在“渲染回调/仿真线程”里调用这个就行：把最新 RGB 帧推给窗口线程显示
    // rgb: 指向 w*h*3 的 RGB 数据（行连续）
    void pushFrame(const uint8_t* rgb, int w, int h) {
        if (!rgb || w <= 0 || h <= 0) return;
        std::lock_guard<std::mutex> lk(frame_mtx_);
        if (w != frame_w_ || h != frame_h_) {
            frame_w_ = w;
            frame_h_ = h;
            frame_.assign((size_t)w * (size_t)h * 3, 0);
        }
        std::memcpy(frame_.data(), rgb, frame_.size());
        // 统计 FPS（每秒更新一次）
        updateFpsLocked();

        // 画 FPS 到左上角（直接改 frame_ 像素）
        overlayFPSLocked();
        frame_dirty_ = true;  // 直接轮训
    }

    void loop(std::atomic<bool>* running_flag = nullptr) {  // 渲染线程调用
        if (!window_) return;
        closed_.store(false);
        makeContextCurrent();
        while (!glfwWindowShouldClose(window_)) {
            if (running_flag && !running_flag->load()) break;
            glfwPollEvents();
            updateInputPerFrame();
            uploadTextureIfNeeded();
            draw();
            glfwSwapBuffers(window_);
        }
        closed_.store(true);
        if (running_flag) running_flag->store(false);
    }

    bool window_once() {  // openGL需要在同一个进程
        if (!window_) return false;
        if (glfwWindowShouldClose(window_)) {
            closed_.store(true);
            return false;
        }
        makeContextCurrent();
        glfwPollEvents();
        updateInputPerFrame();
        uploadTextureIfNeeded();
        draw();
        glfwSwapBuffers(window_);
        return true;
    }

    void shutdown() {
        if (tex_) {
            glDeleteTextures(1, &tex_);
            tex_ = 0;
        }
        if (window_) {
            glfwDestroyWindow(window_);
            window_ = nullptr;
        }
        glfwTerminate();
        closed_.store(true);
    }

   private:
    static void cursorPosCB(GLFWwindow* w, double xpos, double ypos) {
        auto* self = static_cast<GlfwRgbViewer*>(glfwGetWindowUserPointer(w));
        if (!self) return;

        // GLFW 在 CURSOR_DISABLED 模式下，xpos/ypos 仍会变化，我们用 delta 方式
        if (!self->mouse_inited_) {
            self->last_x_ = xpos;
            self->last_y_ = ypos;
            self->mouse_inited_ = true;
            return;
        }
        self->mouse_dx_accum_ += (xpos - self->last_x_);
        self->mouse_dy_accum_ += (ypos - self->last_y_);
        self->last_x_ = xpos;
        self->last_y_ = ypos;
    }

    void updateInputPerFrame() {
        // InputState st;

        std::lock_guard<std::mutex>lk(input_mutex);
        in_state_.w = (glfwGetKey(window_, GLFW_KEY_W) == GLFW_PRESS);
        in_state_.a = (glfwGetKey(window_, GLFW_KEY_A) == GLFW_PRESS);
        in_state_.s = (glfwGetKey(window_, GLFW_KEY_S) == GLFW_PRESS);
        in_state_.d = (glfwGetKey(window_, GLFW_KEY_D) == GLFW_PRESS);

        // 取出鼠标累计 delta（然后清零）
        in_state_.dx = mouse_dx_accum_;
        in_state_.dy = mouse_dy_accum_;
        mouse_dx_accum_ = 0.0;
        mouse_dy_accum_ = 0.0;

        // 简单甩头判定（你可以换成更平滑的滤波/阈值）
        const double th = flick_threshold_;
        in_state_.flickLeft = (in_state_.dx <= -th);
        in_state_.flickRight = (in_state_.dx >= th);
        in_state_.flickUp = (in_state_.dy <= -th);
        in_state_.flickDown = (in_state_.dy >= th);

        // Esc 退出
        if (glfwGetKey(window_, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
            glfwSetWindowShouldClose(window_, 1);
        }

        if (input_cb_) input_cb_(in_state_);
    }

    // InputState 

    void ensureTexture(int w, int h) {
        if (tex_ == 0) glGenTextures(1, &tex_);
        glBindTexture(GL_TEXTURE_2D, tex_);

        // 纹理参数（你也可以用 NEAREST 看起来更“像像素”）
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);

        // 分配纹理存储
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, w, h, 0, GL_RGB,
                     GL_UNSIGNED_BYTE, nullptr);
        tex_w_ = w;
        tex_h_ = h;
    }

    void uploadTextureIfNeeded() {
        std::lock_guard<std::mutex> lk(frame_mtx_);
        if (!frame_dirty_) return;
        if (frame_w_ <= 0 || frame_h_ <= 0 || frame_.empty()) return;

        if (tex_w_ != frame_w_ || tex_h_ != frame_h_) {
            ensureTexture(frame_w_, frame_h_);
        }

        glBindTexture(GL_TEXTURE_2D, tex_);
        // 如果你有 stride（每行字节数）不等于 w*3，需要改成逐行 glTexSubImage2D
        // 或先拷贝成紧凑
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, frame_w_, frame_h_, GL_RGB,
                        GL_UNSIGNED_BYTE, frame_.data());

        frame_dirty_ = false;
    }

    void draw() {
        int ww, wh;
        glfwGetFramebufferSize(window_, &ww, &wh);
        glViewport(0, 0, ww, wh);

        glClearColor(0, 0, 0, 1);
        glClear(GL_COLOR_BUFFER_BIT);

        if (tex_ == 0 || tex_w_ == 0 || tex_h_ == 0) return;

        // 保持图像比例，居中显示
        float winAspect = (wh == 0) ? 1.0f : (float)ww / (float)wh;
        float imgAspect = (float)tex_w_ / (float)tex_h_;

        float sx = 1.0f, sy = 1.0f;
        if (imgAspect > winAspect) {
            // 图更宽：宽铺满，高缩小
            sy = winAspect / imgAspect;
        } else {
            // 图更高：高铺满，宽缩小
            sx = imgAspect / winAspect;
        }

        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        glBindTexture(GL_TEXTURE_2D, tex_);

        // 注意：OpenGL 纹理坐标原点在左下；如果你图像原点在左上，可能会上下颠倒
        // 若颠倒：交换下面 (0,0) 和 (0,1) 的 t 坐标即可
        glBegin(GL_QUADS);
        glTexCoord2f(0.f, 0.f);
        glVertex2f(-sx, -sy);
        glTexCoord2f(1.f, 0.f);
        glVertex2f(sx, -sy);
        glTexCoord2f(1.f, 1.f);
        glVertex2f(sx, sy);
        glTexCoord2f(0.f, 1.f);
        glVertex2f(-sx, sy);
        glEnd();
    }

   private:
    GLFWwindow* window_ = nullptr;

    std::mutex input_mutex;
    InputState in_state_;

    // frame buffer (CPU side)
    std::mutex frame_mtx_;
    std::vector<uint8_t> frame_;
    int frame_w_ = 0;
    int frame_h_ = 0;
    bool frame_dirty_ = false;

    // OpenGL texture
    GLuint tex_ = 0;
    int tex_w_ = 0;
    int tex_h_ = 0;

    // mouse delta accumulator
    bool mouse_inited_ = false;
    double last_x_ = 0.0, last_y_ = 0.0;
    double mouse_dx_accum_ = 0.0;
    double mouse_dy_accum_ = 0.0;
    double flick_threshold_ = 60.0;

  InputCallback input_cb_;
  std::atomic<bool> closed_{false};

  std::chrono::steady_clock::time_point fps_last_ =
      std::chrono::steady_clock::now();
    uint64_t fps_frames_ = 0;
    double fps_value_ = 0.0;
    static void putPixelRGB(std::vector<uint8_t>& img, int w, int h, int x,
                            int y, uint8_t r, uint8_t g, uint8_t b) {
        if (x < 0 || y < 0 || x >= w || y >= h) return;
        size_t idx = ((size_t)y * (size_t)w + (size_t)x) * 3;
        img[idx + 0] = r;
        img[idx + 1] = g;
        img[idx + 2] = b;
    }

    // 将屏幕坐标的 y=0 视为左上角，写入 frame_ 时做垂直翻转
    static void putPixelOverlay(std::vector<uint8_t>& img, int w, int h, int x,
                                int y, uint8_t r, uint8_t g, uint8_t b) {
        if (x < 0 || y < 0 || x >= w || y >= h) return;
        int fy = h - 1 - y;  // flip Y so text不再上下颠倒
        size_t idx = ((size_t)fy * (size_t)w + (size_t)x) * 3;
        img[idx + 0] = r;
        img[idx + 1] = g;
        img[idx + 2] = b;
    }

    // 5x7 字模：每行 5 bit（从高到低），共 7 行
    static const uint8_t* glyph5x7(char c) {
        // 每个 glyph 7 个字节
        static const uint8_t SPACE[7] = {0, 0, 0, 0, 0, 0, 0};
        static const uint8_t COLON[7] = {0x00, 0x04, 0x00, 0x00,
                                         0x04, 0x00, 0x00};  // :
        static const uint8_t DOT[7] = {0x00, 0x00, 0x00, 0x00,
                                       0x00, 0x06, 0x06};  // .

        static const uint8_t F_[7] = {0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x10};
        static const uint8_t P_[7] = {0x1E, 0x11, 0x11, 0x1E, 0x10, 0x10, 0x10};
        static const uint8_t S_[7] = {0x0F, 0x10, 0x10, 0x0E, 0x01, 0x01, 0x1E};

        static const uint8_t D0[7] = {0x0E, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0E};
        static const uint8_t D1[7] = {0x04, 0x0C, 0x04, 0x04, 0x04, 0x04, 0x0E};
        static const uint8_t D2[7] = {0x0E, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1F};
        static const uint8_t D3[7] = {0x1F, 0x02, 0x04, 0x02, 0x01, 0x11, 0x0E};
        static const uint8_t D4[7] = {0x02, 0x06, 0x0A, 0x12, 0x1F, 0x02, 0x02};
        static const uint8_t D5[7] = {0x1F, 0x10, 0x1E, 0x01, 0x01, 0x11, 0x0E};
        static const uint8_t D6[7] = {0x06, 0x08, 0x10, 0x1E, 0x11, 0x11, 0x0E};
        static const uint8_t D7[7] = {0x1F, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08};
        static const uint8_t D8[7] = {0x0E, 0x11, 0x11, 0x0E, 0x11, 0x11, 0x0E};
        static const uint8_t D9[7] = {0x0E, 0x11, 0x11, 0x0F, 0x01, 0x02, 0x0C};

        switch (c) {
            case ' ':
                return SPACE;
            case ':':
                return COLON;
            case '.':
                return DOT;
            case 'F':
                return F_;
            case 'P':
                return P_;
            case 'S':
                return S_;
            case '0':
                return D0;
            case '1':
                return D1;
            case '2':
                return D2;
            case '3':
                return D3;
            case '4':
                return D4;
            case '5':
                return D5;
            case '6':
                return D6;
            case '7':
                return D7;
            case '8':
                return D8;
            case '9':
                return D9;
            default:
                return SPACE;
        }
    }

    void drawText5x7Locked(int x, int y, const char* text, int scale = 2,
                           uint8_t r = 255, uint8_t g = 255, uint8_t b = 255) {
        // 叠一层黑底边（可选，让字更清楚）
        auto drawChar = [&](int cx, int cy, char c, uint8_t rr, uint8_t gg,
                            uint8_t bb) {
            const uint8_t* g5 = glyph5x7(c);
            for (int row = 0; row < 7; ++row) {
                uint8_t bits =
                    g5[row];  // 低 5 位有效（我们用高位也行，这里按 0x1F 设计）
                for (int col = 0; col < 5; ++col) {
                    bool on = (bits & (1u << (4 - col))) != 0;
                    if (!on) continue;
                    for (int sy = 0; sy < scale; ++sy) {
                        for (int sx = 0; sx < scale; ++sx) {
                            putPixelOverlay(frame_, frame_w_, frame_h_,
                                            cx + col * scale + sx,
                                            cy + row * scale + sy, rr, gg, bb);
                        }
                    }
                }
            }
        };

        int pen_x = x;
        for (const char* p = text; *p; ++p) {
            // 先画个黑色描边（简单做法：偏移几次）
            drawChar(pen_x + 1, y + 1, *p, 0, 0, 0);
            drawChar(pen_x, y, *p, r, g, b);
            pen_x += (5 * scale + 1 * scale);
        }
    }

    void updateFpsLocked() {
        fps_frames_++;
        auto now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(now - fps_last_).count();
        if (dt >= 1.0) {
            fps_value_ = fps_frames_ / dt;
            fps_frames_ = 0;
            fps_last_ = now;
        }
    }

    void fillRectLocked(int x, int y, int w, int h, uint8_t r, uint8_t g, uint8_t b) {
        // x,y 是左上角（注意：你的 frame_ 像素坐标 y=0 在最上面还是最下面？
        // 这里假设 frame_ 的 y=0 是“第一行”，也就是左上角。
        // 如果你发现背景跑到左下角，把 y 的坐标改成 frame_h_-y-h 即可。
        for (int yy = 0; yy < h; ++yy) {
            int py = y + yy;
            if (py < 0 || py >= frame_h_) continue;
            for (int xx = 0; xx < w; ++xx) {
            int px = x + xx;
            if (px < 0 || px >= frame_w_) continue;
            putPixelOverlay(frame_, frame_w_, frame_h_, px, py, r, g, b);
            }
        }
    }

    void overlayFPSLocked() {
        char buf[64];
        std::snprintf(buf, sizeof(buf), "FPS: %.1f", fps_value_);

        const int scale = 3;  // 提高清晰度
        const int textW = (int)std::strlen(buf) * (5 * scale + 1 * scale);
        const int textH = 7 * scale;

        const int pad = 8;
        const int x0 = 10;
        const int y0 = 10;

        // 1) 先画背景框（深灰）
        fillRectLocked(x0 - pad, y0 - pad,
                        textW + pad * 2, textH + pad * 2,
                        0, 0, 0);

        // 2) 再画白字（带黑描边）
        drawText5x7Locked(x0, y0, buf, scale, 255, 255, 255);
    }
};
