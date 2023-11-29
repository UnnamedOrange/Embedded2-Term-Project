#include <array>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <iostream>
#include <memory>
#include <thread>

extern "C"
{
#include <fpioa.h>
#include <gpiohs.h>
#include <plic.h>
#include <sysctl.h>

#include "drivers/board_config.h"
#include "drivers/lcd.h"
}

using namespace std::literals;

class Main {
public:
    static constexpr auto LCD_WIDTH = 320;
    static constexpr auto LCD_HEIGHT = 240;

private:
    std::array<uint16_t, LCD_WIDTH * LCD_HEIGHT> g_lcd_gram{};

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

        // TODO.
    }
    void initialize_clock() {
        sysctl_pll_set_freq(SYSCTL_PLL0, 800000000UL);
        sysctl_pll_set_freq(SYSCTL_PLL1, 400000000UL);
        sysctl_clock_enable(SYSCTL_CLOCK_AI);
    }
    void initialize_irq() {
        plic_init();
        // TODO.
    }
    void bitblt() {
        lcd_draw_picture(0, 0, LCD_WIDTH, LCD_HEIGHT, reinterpret_cast<uint32_t*>(g_lcd_gram.data()));
    }
};

int main() {
    [[maybe_unused]] const auto _ = std::make_unique<Main>();
}
