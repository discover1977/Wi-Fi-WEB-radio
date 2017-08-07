			Warning!!!
You need to configure the KaRadio to work with the control card.


Using the UART interface, you need to configure:
1. The speed of the UART is 9600 baud. Command in terminal: sys.uart("9600")
2. Mode of operation of the interface I2S - "1" (96 кГц). Command in terminal: sys.i2s("1")

Using the web interface, enable "Autoplay".

After the settings, restart the KaRadio.