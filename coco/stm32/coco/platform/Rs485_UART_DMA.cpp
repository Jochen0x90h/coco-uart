#include "Rs485_UART_DMA.hpp"
//#include <coco/debug.hpp>


namespace coco {

// Rs485_UART_DMA

Rs485_UART_DMA::~Rs485_UART_DMA() {
}

void Rs485_UART_DMA::startRx(BufferBase &buffer) {
    // don't start receiving if DE is high
    if (this->deState)
        return;

    Uart_UART_DMA::startRx(buffer);
}

void Rs485_UART_DMA::startTx(BufferBase &buffer) {
    // disable receiver
    disableRx();

    // enable DE pin
    gpio::setOutput(this->dePin, true);
    this->deState = true;

    Uart_UART_DMA::startTx(buffer);
}

// called from UART interrupt
void Rs485_UART_DMA::endTx() {
    // disable DE pin
    gpio::setOutput(this->dePin, false);
    this->deState = false;

    // continue receiving
    auto *receiveBuffer = this->receiveTransfers.frontOrNull(); // DMA interrupt has same priority and therefore doesn't need to be disabled
    if (receiveBuffer != nullptr)
        Uart_UART_DMA::startRx(*receiveBuffer);
}

} // namespace coco
