# STM32F070 USB CDC loopback

Data received on the VCP (virtual serial port) are transmitted back to the host.

Changes from the generated STM32CubeMX project (included) are:

# usbd_cdc_if.c

- CDC_Receive_FS() called on receipt of data from host; push data to fifo
- Red LED toggled when incoming packets are lost due to fifo overrun

# main.c

- Top-level loop; unload fifo and call CDC_Transmit_FS() to echo data to host
- Green LED toggled for each buffer pulled from the fifo
