# libkwp71

This is a C++ library implementation of the KWP-71 automotive diagnostic protocol. This protocol first appeared on Bosch Motronic ECUs, and is sometimes also known as KW-71. There are some other protocols that are very similar in design to KWP-71 that this library can support with a change to settings, such as FIAT-9141 and possibly KWP1281.

This library requires a USB K-line adapter that is based on FTDI's FT232R chip, and it also depends on libftdi >= 1.5.
