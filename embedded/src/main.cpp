#include <array>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <iostream>
#include <memory>
#include <thread>

extern "C"
{
#include <dvp.h>
#include <fpioa.h>
#include <gpiohs.h>
#include <plic.h>
#include <sysctl.h>

#include "drivers/board_config.h"
#include "drivers/gc0328.h"
#include "drivers/lcd.h"
}

using namespace std::literals;

class Main {
    using Self = Main;

public:
    static constexpr auto LCD_WIDTH = 320;
    static constexpr auto LCD_HEIGHT = 240;
    static constexpr auto CAMERA_WIDTH = 320;
    static constexpr auto CAMERA_HEIGHT = 240;

private:
    static int irq_dvp(void* ctx) {
        auto& self = *reinterpret_cast<Main*>(ctx);

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

private:
    std::array<uint16_t, LCD_WIDTH * LCD_HEIGHT> lcd_gram{};
    std::array<uint16_t, CAMERA_WIDTH * CAMERA_HEIGHT> dvp_565{};
    std::array<std::array<uint8_t, CAMERA_WIDTH * CAMERA_HEIGHT>, 3> dvp_888_planar{};

    volatile bool has_dvp_finished = true;

public:
    Main() {
        initialize();
    }

    // Private extracted methods.
private:
    void initialize() {
        initialize_clock();
        initialize_power();
        initialize_io();
        initialize_dvp();
        initialize_irq();
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
    }
    void initialize_clock() {
        sysctl_pll_set_freq(SYSCTL_PLL0, 800000000UL);
        sysctl_pll_set_freq(SYSCTL_PLL1, 400000000UL);
        sysctl_clock_enable(SYSCTL_CLOCK_AI);
    }
    void initialize_dvp() {
        dvp_init(8);
        dvp_set_xclk_rate(24000000);
        dvp_enable_burst();
        dvp_set_output_enable(DVP_OUTPUT_AI, 1);
        dvp_set_output_enable(DVP_OUTPUT_DISPLAY, 1);
        dvp_set_image_format(DVP_CFG_RGB_FORMAT);
        dvp_set_image_size(CAMERA_WIDTH, CAMERA_HEIGHT);

        dvp_set_display_addr(reinterpret_cast<uintptr_t>(dvp_565.data()));
        dvp_set_ai_addr(reinterpret_cast<uintptr_t>(dvp_888_planar[0].data()),
                        reinterpret_cast<uintptr_t>(dvp_888_planar[1].data()),
                        reinterpret_cast<uintptr_t>(dvp_888_planar[2].data()));

        gc0328_init();
    }
    void initialize_irq() {
        plic_init();

        // DVP.
        dvp_config_interrupt(DVP_CFG_START_INT_ENABLE | DVP_CFG_FINISH_INT_ENABLE, 0);
        dvp_disable_auto();
        plic_set_priority(IRQN_DVP_INTERRUPT, 1);
        plic_irq_register(IRQN_DVP_INTERRUPT, &Self::irq_dvp, this);
        plic_irq_enable(IRQN_DVP_INTERRUPT);

        sysctl_enable_irq();
    }
    void bitblt() {
        lcd_draw_picture(0, 0, LCD_WIDTH, LCD_HEIGHT, reinterpret_cast<uint32_t*>(lcd_gram.data()));
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
};

int main() {
    [[maybe_unused]] const auto _ = std::make_unique<Main>();
}
