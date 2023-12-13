/**
 * @file main.cpp
 * @author UnnamedOrange
 * @brief Entry.
 * @version 0.1.0
 * @date 2023-12-13
 *
 * @copyright Copyright (c) UnnamedOrange. Licensed under the MIT License.
 * See the LICENSE file in the repository root for full license text.
 */

#include <array>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <iostream>
#include <memory>
#include <optional>
#include <thread>

extern "C"
{
#include <bsp.h>
#include <dvp.h>
#include <fpioa.h>
#include <gpiohs.h>
#include <kpu.h>
#include <plic.h>
#include <pwm.h>
#include <sysctl.h>

#include "drivers/board_config.h"
#include "drivers/gc0328.h"
#include "drivers/lcd.h"

#define INCBIN_STYLE INCBIN_STYLE_SNAKE
#define INCBIN_PREFIX
#include "incbin.h"

    INCBIN(model, "model.kmodel");
}

using namespace std::literals;

class Main {
    using Self = Main;

public:
    static constexpr auto LCD_WIDTH = 320;
    static constexpr auto LCD_HEIGHT = 240;
    static constexpr auto CAMERA_WIDTH = 320;
    static constexpr auto CAMERA_HEIGHT = 240;

    static constexpr auto IR_LED_DUTY = std::array{1.0, 0.7, 0.5, 0.3};
    static constexpr auto JITTER_THRESHOLD = 50ms;
    static constexpr auto HOLD_THRESHOLD = 500ms;
    static constexpr auto HOLD_REPEAT = 20ms;

private:
    static int irq_dvp(void* ctx) {
        auto& self = *reinterpret_cast<Self*>(ctx);

        if (dvp_get_interrupt(DVP_STS_FRAME_FINISH)) {
            dvp_config_interrupt(DVP_CFG_START_INT_ENABLE | DVP_CFG_FINISH_INT_ENABLE, 0);
            dvp_clear_interrupt(DVP_STS_FRAME_FINISH);
            self.has_dvp_finished = true;
        } else {
            dvp_start_convert();
            dvp_clear_interrupt(DVP_STS_FRAME_START);
        }

        return 0;
    }
    static void irq_kpu(void* ctx) {
        auto& self = *reinterpret_cast<Self*>(ctx);

        self.has_kpu_finished = true;
    }
    static int irq_button(void* ctx) {
        auto& self = *reinterpret_cast<Self*>(ctx);

        self.is_button_down = !gpiohs_get_pin(KEY_IO);

        return 0;
    }

    static int core1_routine(void* ctx) { // 天知地知我知。
        auto& self = *reinterpret_cast<Self*>(ctx);

        using Clock = std::chrono::steady_clock;
        auto pre_down = std::optional<Clock::time_point>{};
        auto pre_hold = std::optional<Clock::time_point>{};
        while (true) {
            const auto now = Clock::now();
            if (self.is_button_down) {
                if (!pre_down) {
                    pre_down = std::optional{now};
                }
                const auto duration = now - *pre_down;
                if (duration >= HOLD_THRESHOLD) {
                    if (!pre_hold || now - *pre_hold >= HOLD_REPEAT) {
                        if (!pre_hold) {
                            self.is_increasing = !self.is_increasing;
                        }
                        self.on_button_hold();
                        pre_hold = now;
                    }
                }
            } else {
                if (!pre_down) {
                    continue;
                }
                const auto duration = now - *pre_down;
                if (JITTER_THRESHOLD <= duration && duration < HOLD_THRESHOLD) {
                    self.on_button_click();
                }
                pre_down = std::nullopt;
                pre_hold = std::nullopt;
            }
        }
        return 0;
    }

private:
    std::array<uint16_t, LCD_WIDTH * LCD_HEIGHT> lcd_gram{};
    std::array<uint16_t, CAMERA_WIDTH * CAMERA_HEIGHT> dvp_565{};
    std::array<std::array<uint8_t, CAMERA_WIDTH * CAMERA_HEIGHT>, 3> dvp_888_planar{};

    std::array<uint8_t, CAMERA_WIDTH * CAMERA_HEIGHT> mask{};
    std::array<uint32_t, CAMERA_WIDTH * CAMERA_HEIGHT> sum{};
    std::array<std::array<uint8_t, CAMERA_WIDTH * CAMERA_HEIGHT>, 3> blurred{};

    kpu_model_context_t model_context;

    volatile bool has_dvp_finished = true;
    volatile bool has_kpu_finished = true;
    volatile bool is_button_down = false;

    size_t ir_led_duty_index_plus_1 = 1; // 0 for not enabled.
    uint8_t background_threshold = 35;
    bool is_increasing = false;

public:
    Main() {
        initialize();

        while (true) {
            capture_blocking(0);
            dvp_565_to_dvp_888_planar();
            calculate_mask();

            capture_blocking(1);
            dvp_565_to_dvp_888_planar();
            if (ir_led_duty_index_plus_1) {
                calc_blurred();
                calc_blurred_soft(mask, mask, 7);
                apply_blurred_to_dvp_888();
            }
            dvp_888_planar_to_dvp_565();

            lcd_gram = dvp_565;
            bitblt();
        }
    }

    // Private extracted methods.
private:
    void initialize() {
        initialize_clock();
        initialize_power();
        initialize_io();
        initialize_dvp();
        initialize_model();
        initialize_irq();
        register_core1(&Self::core1_routine, this);
    }
    void initialize_power() {
        sysctl_set_power_mode(SYSCTL_POWER_BANK6, SYSCTL_POWER_V18);
        sysctl_set_power_mode(SYSCTL_POWER_BANK7, SYSCTL_POWER_V18);
    }
    void initialize_io() {
        // LCD SPI.
        fpioa_set_function(LCD_DC_PIN, fpioa_function_t(int(FUNC_GPIOHS0) + LCD_DC_IO));
        fpioa_set_function(LCD_CS_PIN, FUNC_SPI0_SS3);
        fpioa_set_function(LCD_RW_PIN, FUNC_SPI0_SCLK);
        fpioa_set_function(LCD_RST_PIN, fpioa_function_t(int(FUNC_GPIOHS0) + LCD_RST_IO));
        sysctl_set_spi0_dvp_data(1);

        // LCD initialization.
        lcd_init();
        lcd_set_direction(DIR_YX_RLDU);
        bitblt();
        std::this_thread::sleep_for(32ms);

        // LCD backlight.
        fpioa_set_function(LCD_BLIGHT_PIN, fpioa_function_t(int(FUNC_GPIOHS0) + LCD_BLIGHT_IO));
        gpiohs_set_drive_mode(LCD_BLIGHT_IO, GPIO_DM_OUTPUT);
        gpiohs_set_pin(LCD_BLIGHT_IO, GPIO_PV_LOW);

        // DVP.
        fpioa_set_function(DVP_PWDN_PIN, FUNC_CMOS_PWDN);
        fpioa_set_function(DVP_XCLK_PIN, FUNC_CMOS_XCLK);
        fpioa_set_function(DVP_VSYNC_PIN, FUNC_CMOS_VSYNC);
        fpioa_set_function(DVP_HREF_PIN, FUNC_CMOS_HREF);
        fpioa_set_function(DVP_PCLK_PIN, FUNC_CMOS_PCLK);
        fpioa_set_function(DVP_SCCB_SCLK_PIN, FUNC_SCCB_SCLK);
        fpioa_set_function(DVP_SCCB_SDA_PIN, FUNC_SCCB_SDA);

        // Infrared LED.
        fpioa_set_function(LED_IR_PIN, FUNC_TIMER0_TOGGLE1);
        pwm_init(PWM_DEVICE_0);
        update_ir_led();
        pwm_set_enable(PWM_DEVICE_0, PWM_CHANNEL_0, true);

        // Button.
        fpioa_set_function(KEY_PIN, fpioa_function_t(int(FUNC_GPIOHS0) + KEY_IO));
        gpiohs_set_drive_mode(KEY_IO, GPIO_DM_INPUT_PULL_UP);
        gpiohs_set_pin_edge(KEY_IO, GPIO_PE_BOTH);
    }
    void initialize_clock() {
        sysctl_pll_set_freq(SYSCTL_PLL0, 800000000UL);
        sysctl_pll_set_freq(SYSCTL_PLL1, 400000000UL);
        sysctl_clock_enable(SYSCTL_CLOCK_AI);
    }
    void initialize_dvp() {
        dvp_init(8);
        dvp_set_xclk_rate(64000000);
        dvp_enable_burst();
        dvp_set_output_enable(DVP_OUTPUT_AI, 0);
        dvp_set_output_enable(DVP_OUTPUT_DISPLAY, 1);
        dvp_set_image_format(DVP_CFG_RGB_FORMAT);
        dvp_set_image_size(CAMERA_WIDTH, CAMERA_HEIGHT);

        dvp_set_display_addr(reinterpret_cast<uintptr_t>(dvp_565.data()));
        dvp_set_ai_addr(reinterpret_cast<uintptr_t>(dvp_888_planar[0].data()),
                        reinterpret_cast<uintptr_t>(dvp_888_planar[1].data()),
                        reinterpret_cast<uintptr_t>(dvp_888_planar[2].data()));

        gc0328_init();
    }
    void initialize_model() {
        kpu_load_kmodel(&model_context, model_data);
    }
    void initialize_irq() {
        plic_init();

        // DVP.
        dvp_config_interrupt(DVP_CFG_START_INT_ENABLE | DVP_CFG_FINISH_INT_ENABLE, 0);
        dvp_disable_auto();
        plic_set_priority(IRQN_DVP_INTERRUPT, 1);
        plic_irq_register(IRQN_DVP_INTERRUPT, &Self::irq_dvp, this);
        plic_irq_enable(IRQN_DVP_INTERRUPT);

        // Button.
        gpiohs_irq_register(KEY_IO, 1, &Self::irq_button, this);

        sysctl_enable_irq();
    }
    void update_ir_led() {
        if (ir_led_duty_index_plus_1) {
            pwm_set_frequency(PWM_DEVICE_0, PWM_CHANNEL_0, 3000, IR_LED_DUTY[ir_led_duty_index_plus_1 - 1]);
        } else {
            pwm_set_frequency(PWM_DEVICE_0, PWM_CHANNEL_0, 3000, 0.0);
        }
    }
    void on_button_click() {
        if (ir_led_duty_index_plus_1 >= IR_LED_DUTY.size()) {
            ir_led_duty_index_plus_1 = 0;
        } else {
            ir_led_duty_index_plus_1++;
        }
        update_ir_led();
    }
    void on_button_hold() {
        constexpr uint16_t min = 0;
        constexpr uint16_t max = 255;
        if (is_increasing) {
            if (background_threshold >= max) {
                is_increasing = false;
            } else {
                background_threshold++;
            }
        } else {
            if (background_threshold <= min) {
                is_increasing = true;
            } else {
                background_threshold--;
            }
        }
    }
    void bitblt() {
        lcd_draw_picture(0, 0, LCD_WIDTH, LCD_HEIGHT, reinterpret_cast<uint32_t*>(lcd_gram.data()));
    }
    void dvp_565_to_dvp_888_planar() {
        for (auto i = 0; i < CAMERA_HEIGHT; i++) {
            for (auto j = 0; j < CAMERA_WIDTH; j++) {
                const auto idx = i * CAMERA_WIDTH + j;
                const uint16_t pixel = dvp_565[idx];
                const uint8_t r = (pixel >> 11) & ((1 << 5) - 1);
                const uint8_t g = (pixel >> 5) & ((1 << 6) - 1);
                const uint8_t b = (pixel >> 0) & ((1 << 5) - 1);
                dvp_888_planar[0][idx] = (r << (8 - 5)) | (r >> (5 - (8 - 5)));
                dvp_888_planar[1][idx] = (g << (8 - 6)) | (g >> (6 - (8 - 6)));
                dvp_888_planar[2][idx] = (b << (8 - 5)) | (b >> (5 - (8 - 5)));
            }
        }
    }
    void dvp_888_planar_to_dvp_565() {
        for (auto i = 0; i < CAMERA_HEIGHT; i++) {
            for (auto j = 0; j < CAMERA_WIDTH; j++) {
                const auto idx = i * CAMERA_WIDTH + j;
                const uint16_t r = dvp_888_planar[0][idx];
                const uint16_t g = dvp_888_planar[1][idx];
                const uint16_t b = dvp_888_planar[2][idx];
                dvp_565[idx] = (b >> (8 - 5)) | (g >> (8 - 6) << 5) | (r >> (8 - 5) << 11);
            }
        }
    }
    void capture_blocking(int camera_id) {
        if (camera_id == 0) {
            open_gc0328_0();
        } else {
            open_gc0328_1();
        }

        has_dvp_finished = false;
        dvp_clear_interrupt(DVP_STS_FRAME_START | DVP_STS_FRAME_FINISH);
        dvp_config_interrupt(DVP_CFG_START_INT_ENABLE | DVP_CFG_FINISH_INT_ENABLE, 1);
        while (!has_dvp_finished)
            ;
    }
    void run_kpu() {
        if (has_kpu_finished) {
            has_kpu_finished = false;
            kpu_run_kmodel(&model_context, dvp_888_planar[0].data(), DMAC_CHANNEL5, &Self::irq_kpu, this);
        }
    }
    void wait_for_kpu() {
        while (!has_kpu_finished)
            ;
    }
    void run_kpu_blocking() {
        run_kpu();
        wait_for_kpu();
    }
    void calculate_mask() {
        const auto threshold = background_threshold;
        // Assume it is IR.
        for (auto i = 0; i < CAMERA_HEIGHT; i++) {
            for (auto j = 0; j < CAMERA_WIDTH; j++) {
                const auto idx = i * CAMERA_WIDTH + j;
                uint8_t min = 255;
                uint8_t max = 0;
                for (auto k = 0; k < 3; k++) {
                    min = std::min(min, dvp_888_planar[k][idx]);
                    max = std::max(max, dvp_888_planar[k][idx]);
                }
                const auto brightness = (int(max) + int(min)) / 2;
                if (brightness >= threshold) {
                    mask[idx] = 255;
                } else {
                    mask[idx] = 64 * (threshold - brightness) / threshold;
                }
            }
        }
    }
    void calc_blurred() {
        for (auto i = 0; i < 3; i++) {
            calc_blurred_soft(dvp_888_planar[i], blurred[i], 35);
        }
    }
    void calc_blurred_soft(const std::array<uint8_t, CAMERA_WIDTH * CAMERA_HEIGHT>& src, //
                           std::array<uint8_t, CAMERA_WIDTH * CAMERA_HEIGHT>& des,       //
                           int radius) {
        for (int y = 0; y < CAMERA_HEIGHT; ++y) {
            uint32_t sumR = 0;
            for (int x = 0; x < radius; x++) {
                sumR += src[y * CAMERA_WIDTH + x];
            }
            for (int x = 0; x < CAMERA_WIDTH; ++x) {
                sumR += src[y * CAMERA_WIDTH + x];
                if (x >= radius) {
                    sumR -= src[y * CAMERA_WIDTH + (x - radius)];
                } else {
                    sumR -= src[y * CAMERA_WIDTH + (radius - x - 1)];
                }
                sum[y * CAMERA_WIDTH + x] = sumR;
            }
        }
        for (int x = 0; x < CAMERA_WIDTH; ++x) {
            uint32_t sumR = 0;
            for (int y = 0; y < radius; y++) {
                sumR += sum[y * CAMERA_WIDTH + x];
            }
            for (int y = 0; y < CAMERA_HEIGHT; ++y) {
                sumR += sum[y * CAMERA_WIDTH + x];
                if (y >= radius) {
                    sumR -= sum[(y - radius) * CAMERA_WIDTH + x];
                } else {
                    sumR -= sum[(radius - y - 1) * CAMERA_WIDTH + x];
                }
                des[y * CAMERA_WIDTH + x] = static_cast<uint8_t>(sumR / (radius * radius));
            }
        }
    }
    void apply_blurred_to_dvp_888() {
        for (auto i = 0; i < CAMERA_HEIGHT; i++) {
            for (auto j = 0; j < CAMERA_WIDTH; j++) {
                const auto idx = i * CAMERA_WIDTH + j;
                const auto m = mask[idx];
                for (auto k = 0; k < 3; k++) {
                    auto& target = dvp_888_planar[k][idx];
                    const auto& b = blurred[k][idx];
                    target = static_cast<uint8_t>((uint16_t(target) * m + uint16_t(b) * (255 - m)) / 255);
                }
            }
        }
    }
};

int main() {
    [[maybe_unused]] const auto _ = std::make_unique<Main>();
}
