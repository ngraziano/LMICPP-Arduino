
enum class Sleep : unsigned char
{
    P15MS,
    P30MS,
    P60MS,
    P120MS,
    P250MS,
    P500MS,
    P1S,
    P2S,
    P4S,
    P8S,
    FOREVER
};

void powerDown(Sleep period);
void configure_wdt();
void rst_wdt();

