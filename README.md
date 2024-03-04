# libiceblock

This is a C++ library implementation of the KWP-71 automotive diagnostic protocol, along with other protocols that have a similar design based on block exchange. The KWP-71 protocol first appeared on Bosch Motronic ECUs, and is sometimes also known as KW-71 or "Keyword 71".

FIAT-9141 and Marelli 1AF are similar enough to KWP-71 to be easily supported, but KW82 has much more complex timing requirements and it probably will not be included.

This library requires a USB K-line adapter that is based on FTDI's FT232R chip, and it also depends on libftdi >= 1.5.

