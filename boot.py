# boot.py -- run on boot-up
import network
import machine


def connect_wifi(ssid, password):
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect(ssid, password)
    while not wlan.isconnected():
        machine.idle()


def main():
    SSID = "DIGIFIBRA-36Yz"
    PASSWORD = "G2TfEHtNRbEk"
    # SSID = "DIGIFIBRA-yxZ5"
    # PASSWORD = "bfEZ2yDyc57d"
    #SSID = "udcdocencia"
    #PASSWORD = "Universidade.2223"
    connect_wifi(SSID, PASSWORD)
    print("WIfi conectado.")


if __name__ == "__main__":
    main()
