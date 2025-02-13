# libiceblock

This is a C++ library implementation of early and obscure software protocols that are used for diagnostics on automotive ECUs. The focus of this project is mainly on protocols that predate the OBD-II era (which started with model year 1996 in the US.)

Many of these protocols are of the "block exchange" type, which require that the tester software and the ECU take turns sending frames of data. When the connection is idle, these frames will simply be of an ACK or NOP type.

An early progenitor of these block-based schemes is Keyword Protocol 71 (also known as KWP-71 or KP-71), which was developed by Bosch in the 1980s and was implemented by their Motronic engine management ECUs until the mid-1990s. FIAT-9141 and Marelli 1AF are similar enough to KWP-71 to be easily supported, but KW82 has much more complex timing requirements and it probably will not be included.

This library requires a USB K-line adapter that is based on FTDI's FT232R chip, and it also depends on libftdi >= 1.5. Some assumptions are made about how the FTDI signal lines are connected to the K and L-lines.

## Supported protocols

* KWP-71
* FIAT-9141
* Marelli 1AF

