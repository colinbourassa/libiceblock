# libiceblock

This is a C++ library implementation of the KWP-71 automotive diagnostic protocol, along with other protocols that have a similar design based on block exchange. The KWP-71 protocol first appeared on Bosch Motronic ECUs, and is sometimes also known as KW-71 or "Keyword 71".

This library requires a USB K-line adapter that is based on FTDI's FT232R chip, and it also depends on libftdi >= 1.5.

