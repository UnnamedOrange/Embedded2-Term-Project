#include <cstdio>
#include <iostream>
#include <memory>

extern "C"
{
#include <plic.h>
#include <sysctl.h>
}

class Main {
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
};

int main() {
    [[maybe_unused]] const auto _ = std::make_unique<Main>();
}
