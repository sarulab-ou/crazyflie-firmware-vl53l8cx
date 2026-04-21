
#include "deck.h"

/*ST includes */
#include "stm32fxxx.h"
#include "config.h"

#include "FreeRTOS.h"
#include "semphr.h"

#include "cfassert.h"
#include "config.h"
#include "nvicconf.h"

#define SPI                     SPI1
#define SPI_CLK                 RCC_APB2Periph_SPI1
#define SPI_CLK_INIT            RCC_APB2PeriphClockCmd
#define SPI_IRQ_HANDLER         SPI1_IRQHandler
#define SPI_IRQn                SPI1_IRQn

// #define SPI_DMA_IRQ_PRIO        (NVIC_HIGH_PRI)
// #define SPI_DMA                 DMA2
// #define SPI_DMA_CLK             RCC_AHB1Periph_DMA2
// #define SPI_DMA_CLK_INIT        RCC_AHB1PeriphClockCmd

// #define SPI_TX_DMA_STREAM       DMA2_Stream5
// #define SPI_TX_DMA_IRQ          DMA2_Stream5_IRQn
// #define SPI_TX_DMA_IRQHandler   DMA2_Stream5_IRQHandler
// #define SPI_TX_DMA_CHANNEL      DMA_Channel_3
// #define SPI_TX_DMA_FLAG_TCIF    DMA_FLAG_TCIF5

// #define SPI_RX_DMA_STREAM       DMA2_Stream0
// #define SPI_RX_DMA_IRQ          DMA2_Stream0_IRQn
// #define SPI_RX_DMA_IRQHandler   DMA2_Stream0_IRQHandler
// #define SPI_RX_DMA_CHANNEL      DMA_Channel_3
// #define SPI_RX_DMA_FLAG_TCIF    DMA_FLAG_TCIF0

#define SPI_SCK_PIN             GPIO_Pin_5
#define SPI_SCK_GPIO_PORT       GPIOA
#define SPI_SCK_GPIO_CLK        RCC_AHB1Periph_GPIOA
#define SPI_SCK_SOURCE          GPIO_PinSource5
#define SPI_SCK_AF              GPIO_AF_SPI1

#define SPI_MISO_PIN            GPIO_Pin_6
#define SPI_MISO_GPIO_PORT      GPIOA
#define SPI_MISO_GPIO_CLK       RCC_AHB1Periph_GPIOA
#define SPI_MISO_SOURCE         GPIO_PinSource6
#define SPI_MISO_AF             GPIO_AF_SPI1

#define SPI_MOSI_PIN            GPIO_Pin_7
#define SPI_MOSI_GPIO_PORT      GPIOA
#define SPI_MOSI_GPIO_CLK       RCC_AHB1Periph_GPIOA
#define SPI_MOSI_SOURCE         GPIO_PinSource7
#define SPI_MOSI_AF             GPIO_AF_SPI1


#define DUMMY_BYTE         0xA5

#define DECK_SPI_MODE3

static bool isInit = false;

// static SemaphoreHandle_t txComplete;
// static SemaphoreHandle_t rxComplete;
// static SemaphoreHandle_t spiMutex;

static void spiConfigureWithSpeed(uint16_t baudRatePrescaler);


void spiBegin(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  // binary semaphores created using xSemaphoreCreateBinary() are created in a state
  // such that the semaphore must first be 'given' before it can be 'taken'
  // txComplete = xSemaphoreCreateBinary();
  // rxComplete = xSemaphoreCreateBinary();
  // spiMutex = xSemaphoreCreateMutex();

  /*!< Enable the SPI clock */
  SPI_CLK_INIT(SPI_CLK, ENABLE);

  /*!< Enable GPIO clocks */
  RCC_AHB1PeriphClockCmd(SPI_SCK_GPIO_CLK | SPI_MISO_GPIO_CLK |
                         SPI_MOSI_GPIO_CLK, ENABLE);


  /*!< SPI pins configuration *************************************************/

  /*!< Connect SPI pins to AF5 */
  GPIO_PinAFConfig(SPI_SCK_GPIO_PORT, SPI_SCK_SOURCE, SPI_SCK_AF);
  GPIO_PinAFConfig(SPI_MISO_GPIO_PORT, SPI_MISO_SOURCE, SPI_MISO_AF);
  GPIO_PinAFConfig(SPI_MOSI_GPIO_PORT, SPI_MOSI_SOURCE, SPI_MOSI_AF);

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
#ifdef DECK_SPI_MODE3
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
#else
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
#endif

  /*!< SPI SCK pin configuration */
  GPIO_InitStructure.GPIO_Pin = SPI_SCK_PIN;
  GPIO_Init(SPI_SCK_GPIO_PORT, &GPIO_InitStructure);

  /*!< SPI MOSI pin configuration */
  GPIO_InitStructure.GPIO_Pin =  SPI_MOSI_PIN;
  GPIO_Init(SPI_MOSI_GPIO_PORT, &GPIO_InitStructure);

  /*!< SPI MISO pin configuration */
  GPIO_InitStructure.GPIO_Pin =  SPI_MISO_PIN;
  GPIO_Init(SPI_MISO_GPIO_PORT, &GPIO_InitStructure);

  /*!< SPI configuration */
  spiConfigureWithSpeed(SPI_BAUDRATE_2MHZ);

  isInit = true;
}

static void spiConfigureWithSpeed(uint16_t baudRatePrescaler)
{
  SPI_InitTypeDef  SPI_InitStructure;

  SPI_I2S_DeInit(SPI);

  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
#ifdef DECK_SPI_MODE3
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
#else
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
#endif
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 0; // Not used

  SPI_InitStructure.SPI_BaudRatePrescaler = baudRatePrescaler;
  SPI_Init(SPI, &SPI_InitStructure);
}

bool spiTest(void)
{
  return isInit;
}

bool spiExchange(size_t length, const uint8_t * data_tx, uint8_t * data_rx)
{
  // Enable peripheral
  SPI_Cmd(SPI, ENABLE);

  // // Wait for completion
  // bool result = (xSemaphoreTake(txComplete, portMAX_DELAY) == pdTRUE)
  //            && (xSemaphoreTake(rxComplete, portMAX_DELAY) == pdTRUE);

  for (size_t i = 0; i < length; i++)
  {
    // Wait until the transmit buffer is empty
    while (SPI_I2S_GetFlagStatus(SPI, SPI_I2S_FLAG_TXE) == RESET);

    // Send byte through the SPI peripheral
    SPI_I2S_SendData(SPI, data_tx ? data_tx[i] : DUMMY_BYTE);

    // Wait to receive a byte
    while (SPI_I2S_GetFlagStatus(SPI, SPI_I2S_FLAG_RXNE) == RESET);

    // Return the byte read from the SPI bus
    if (data_rx)
    {
      data_rx[i] = SPI_I2S_ReceiveData(SPI);
    }
    else
    {
      // If the caller doesn't care about the received data, just read and discard it
      (void)SPI_I2S_ReceiveData(SPI);
    }
  }
  // Disable peripheral
  SPI_Cmd(SPI, DISABLE);
  return true;
}

void spiBeginTransaction(uint16_t baudRatePrescaler)
{
  // xSemaphoreTake(spiMutex, portMAX_DELAY);
  spiConfigureWithSpeed(baudRatePrescaler);
}

void spiEndTransaction()
{
  // xSemaphoreGive(spiMutex);
}
