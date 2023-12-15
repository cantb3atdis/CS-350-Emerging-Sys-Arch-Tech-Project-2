#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <ti/drivers/GPIO.h>
#include "ti_drivers_config.h"

#include <ti/drivers/I2C.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/Timer.h>

#define DISPLAY(x) UART_write(uart, &output, x);

// UART Global Variables
char output[64];
int bytesToSend;

// Driver Handles - Global variables
I2C_Handle i2c;

// Driver Handles - Global variables
UART_Handle uart;

// Driver Handles - Global variables
Timer_Handle timer0;

volatile unsigned char TimerFlag = 0;

bool buttonFlag0 = false;
bool buttonFlag1 = false;
bool heat = false;
int16_t setpoint = 0;  // Initialize setpoint temperature
int16_t temperature = 0;
unsigned int seconds = 0;

// I2C Global Variables
static const struct
{
    uint8_t address;
    uint8_t resultReg;
    char *id;
} sensors[3] = { { 0x48, 0x0000, "11X" }, { 0x49, 0x0000, "116" }, { 0x41,
                                                                     0x0001,
                                                                     "006" } };
uint8_t txBuffer[1];
uint8_t rxBuffer[2];
I2C_Transaction i2cTransaction;

// Timer callback
void timerCallback(Timer_Handle myHandle, int_fast16_t status)
{
    TimerFlag = 1;
}

void gpioButtonFxn0(uint_least8_t index)
{

    setpoint++; // increase setpoint
}

void gpioButtonFxn1(uint_least8_t index)
{
    setpoint--; // decrease setpoint
}

void initUART(void)
{
    UART_Params uartParams;

    // Init the driver
    UART_init();

    // Configure the driver
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.baudRate = 115200;

    // Open the driver
    uart = UART_open(CONFIG_UART_0, &uartParams);

    if (uart == NULL)
    {
        /* UART_open() failed */
        while (1)
            ;
    }
}

void initI2C(void)
{
    int8_t i, found;
    I2C_Params i2cParams;
    DISPLAY(snprintf(output, 64, "Initializing I2C Driver - "))

    I2C_init();

    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;

    i2c = I2C_open(CONFIG_I2C_0, &i2cParams);
    if (i2c == NULL)
    {
        DISPLAY(snprintf(output, 64, "Failed\n\r"))
        while (1)
            ;
    }

    DISPLAY(snprintf(output, 32, "Passed\n\r"))

    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 0;

    found = false;
    for (i = 0; i < 3; ++i)
    {
        i2cTransaction.slaveAddress = sensors[i].address;
        txBuffer[0] = sensors[i].resultReg;

        DISPLAY(snprintf(output, 64, "Is this %s? ", sensors[i].id))

        if (I2C_transfer(i2c, &i2cTransaction))
        {
            DISPLAY(snprintf(output, 64, "Found\n\r"))
            found = true;
            break;
        }

        DISPLAY(snprintf(output, 64, "No\n\r"))
    }

    if (found)
    {
        DISPLAY(snprintf(output, 64, "Detected TMP%s I2C address: %x\n\r",
                         sensors[i].id, i2cTransaction.slaveAddress))
    }
    else
    {
        DISPLAY(snprintf(output, 64,
                         "Temperature sensor not found, contact professor\n\r"))
    }
}

int16_t readTemp(void)
{
    int16_t temperature = 0;

    i2cTransaction.readCount = 2;
    if (I2C_transfer(i2c, &i2cTransaction))
    {
        temperature = (rxBuffer[0] << 8) | (rxBuffer[1]);
        temperature *= 0.0078125;

        if (rxBuffer[0] & 0x80)
        {
            temperature |= 0xF000;
        }
    }
    else
    {
        DISPLAY(snprintf(output, 64,
                         "Error reading temperature sensor (%d)\n\r",
                         i2cTransaction.status))
        DISPLAY(snprintf(
                output,
                64,
                "Please power cycle your board by unplugging USB and plugging back in.\n\r"))
    }
    return temperature;
}

void initTimer(void)
{
    Timer_Params params;

    Timer_init();
    Timer_Params_init(&params);
    params.period = 1000000;
    params.periodUnits = Timer_PERIOD_US;
    params.timerMode = Timer_CONTINUOUS_CALLBACK;
    params.timerCallback = timerCallback;
    timer0 = Timer_open(CONFIG_TIMER_0, &params);

    if (timer0 == NULL)
    {
        while (1)
        {
        }
    }
    if (Timer_start(timer0) == Timer_STATUS_ERROR)
    {
        while (1)
        {
        }
    }
}

void* mainThread(void *arg0)
{
    GPIO_init();

    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);

    GPIO_setConfig(CONFIG_GPIO_BUTTON_0,
                   GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);

    GPIO_setCallback(CONFIG_GPIO_BUTTON_0, gpioButtonFxn0);

    GPIO_enableInt(CONFIG_GPIO_BUTTON_0);

    if (CONFIG_GPIO_BUTTON_0 != CONFIG_GPIO_BUTTON_1)
    {
        /* Configure BUTTON1 pin */
        GPIO_setConfig(CONFIG_GPIO_BUTTON_1,
                       GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

        /* Install Button callback */
        GPIO_setCallback(CONFIG_GPIO_BUTTON_1, gpioButtonFxn1);
        GPIO_enableInt(CONFIG_GPIO_BUTTON_1);
    }

    initUART();
    initI2C();
    initTimer();

    uint16_t previousTemperature = 0;
    uint16_t previousSetpoint = 0;
    uint16_t previousSeconds = 0;

    while (1)
    {
        if (TimerFlag)
        {
            // Check the buttons every 200ms
            if (buttonFlag0)
            {
                setpoint++;  // Increase setpoint
                buttonFlag0 = 0;
            }
            if (buttonFlag1)
            {
                setpoint--;  // Decrease setpoint
                buttonFlag1 = 0;
            }

            // Read temperature and update LED state every 500ms
            temperature = readTemp();
            if (temperature > setpoint)
            {
                heat = false;
                // Turn off LED (heater)
                GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
            }
            else
            {
                heat = true;
                // Turn on LED (heater)
                GPIO_write(CONFIG_GPIO_LED_0, !CONFIG_GPIO_LED_ON);
            }

            // Report to the server every second via UART
            if (seconds != previousSeconds || temperature != previousTemperature
                    || setpoint != previousSetpoint)
            {

                DISPLAY(snprintf(output, 64, "<%02d,%02d,%d,%04d>\n\r",
                                 temperature, setpoint, heat ? 1 : 0, seconds));

                previousTemperature = temperature;
                previousSetpoint = setpoint;
                previousSeconds = seconds;
            }

            TimerFlag = 0;
        }
    }
}
