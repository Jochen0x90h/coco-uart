#include "Rs485_UART_DMA.hpp"
//#include <coco/debug.hpp>
//#include <coco/StreamOperators.hpp>


namespace coco {

// Rs485_UART_DMA

Rs485_UART_DMA::~Rs485_UART_DMA() {
}

void Rs485_UART_DMA::startRx(BufferBase &buffer) {
    // don't start receiving if DE is high
    if (deState_)
        return;

    Uart_UART_DMA::startRx(buffer);
}

// called when interrupts are disalbed (via Uart_UART_DMA::BufferBase::start() or Uart_UART_DMA::handleTx())
void Rs485_UART_DMA::startTx(BufferBase &buffer) {
    //debug::out << "startTx1\n";

    // disable receiver
    disableRx();

    // enable DE pin
    //debug::out << "startTx2\n";
    gpio::setOutput(dePin_, true);
    deState_ = true;

    Uart_UART_DMA::startTx(buffer);
}

// called when interrupts are disalbed (via Uart_UART_DMA::BufferBase::cancel() or Uart_UART_DMA::handleTx())
void Rs485_UART_DMA::endTx() {
    // disable DE pin
    gpio::setOutput(dePin_, false);
    //debug::out << "endTx1\n";
    deState_ = false;

    // continue receiving
    auto *receiveBuffer = receiveTransfers_.frontOrNull(); // DMA interrupt has same priority and therefore doesn't need to be disabled
    if (receiveBuffer != nullptr)
        Uart_UART_DMA::startRx(*receiveBuffer);
}

} // namespace coco
