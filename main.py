from umqtt.simple import MQTTClient
import ssl
import ntptime
import math
import utime
import json
import machine
from machine import Pin, ADC


# Definicin de pines para el Arduino Nano ESP32
VBATT_IN_PIN = 1  # ADC1_CH0 (GPIO 1 / A0)
AMPS_IN_PIN = 2  # ADC1_CH1 (GPIO 2 / A1)
THROTTLE_IN_PIN = 3  # ADC1_CH2 (GPIO 3 / A2)
TEMP1_IN_PIN = 4  # ADC1_CH3 (GPIO 4 / A3)
TEMP2_IN_PIN = 11  # ADC2_CH0 (GPIO 11 / A4)
VBATT1_IN_PIN = 12  # ADC2_CH1 (GPIO 12 / A5)

BRAKE_IN_PIN = 14  # GPIO 14
LAUNCH_BTN_IN_PIN = 5  # GPIO 5
CYCLE_BTN_IN_PIN = 6  # GPIO 6
MOTOR_RPM_PIN = 7  # GPIO 7
WHEEL_RPM_PIN = 13  # GPIO 13
MOTOR_OUT_PIN = 12  # GPIO 12


# Configuracin de pines
motor_out = Pin(MOTOR_OUT_PIN, Pin.OUT)
motor_out.value(0)

vbatt_in = ADC(Pin(VBATT_IN_PIN))
vbatt_in.atten(ADC.ATTN_11DB)  # Configura la atenuacin para el rango de 0-3.3V
throttle_in = ADC(Pin(THROTTLE_IN_PIN))
throttle_in.atten(ADC.ATTN_11DB)
amps_in = ADC(Pin(AMPS_IN_PIN))
amps_in.atten(ADC.ATTN_11DB)
temp1_in = ADC(Pin(TEMP1_IN_PIN))
temp1_in.atten(ADC.ATTN_11DB)
temp2_in = ADC(Pin(TEMP2_IN_PIN))
temp2_in.atten(ADC.ATTN_11DB)
vbatt1_in = ADC(Pin(VBATT1_IN_PIN))
vbatt1_in.atten(ADC.ATTN_11DB)

brake_in = Pin(BRAKE_IN_PIN, Pin.IN, Pin.PULL_UP)
launch_btn_in = Pin(LAUNCH_BTN_IN_PIN, Pin.IN, Pin.PULL_UP)
cycle_btn_in = Pin(CYCLE_BTN_IN_PIN, Pin.IN, Pin.PULL_UP)


# Configuracin pines

motor_out = machine.Pin(MOTOR_OUT_PIN, machine.Pin.OUT)
motor_out.value(0)

vbatt_in = machine.ADC(machine.Pin(VBATT_IN_PIN))
throttle_in = machine.ADC(machine.Pin(THROTTLE_IN_PIN))
amps_in = machine.ADC(machine.Pin(AMPS_IN_PIN))
temp1_in = machine.ADC(machine.Pin(TEMP1_IN_PIN))
temp2_in = temp1_in
vbatt1_in = amps_in

brake_in = machine.Pin(BRAKE_IN_PIN, machine.Pin.IN, machine.Pin.PULL_UP)
launch_btn_in = machine.Pin(LAUNCH_BTN_IN_PIN, machine.Pin.IN, machine.Pin.PULL_UP)
cycle_btn_in = machine.Pin(CYCLE_BTN_IN_PIN, machine.Pin.IN, machine.Pin.PULL_UP)

# Variables globales

lastShortDataSendTime = 0
lastWheelSpeedPollTime = 0
lastMotorSpeedPollTime = 0
currentSensorOffset = 0
loopCounter = 0
motorPoll = 0
wheelPoll = 0

# Definicin de constantes
AMPSENSOR_CAL_DELAY = 3000
CAL_WHEEL_MAGNETS = 6
CAL_MOTOR_MAGNETS = 3
CAL_WHEEL_CIRCUMFERENCE = 1.178
CAL_REFERENCE_VOLTAGE = 5.0
CAL_BATTERY_TOTAL = 6.15
CAL_BATTERY_LOWER = 3.071
CAL_CURRENT = 37.55

CAL_THERM_A = 0.001871300068
CAL_THERM_B = 0.00009436080271
CAL_THERM_C = 0.0000007954800125


# Configuracin de MQTT

MQTT_PORT = 8883
MQTT_TOPIC = "PiPico"


def sync_time():
    print("Syncing time...")
    ntptime.settime()
    print("Time synchronized")


def connect():
    global client, TOPIC
    CLIENT_ID = "basicPubSub"
    SERVER = "a3k4s8s6rf8peu-ats.iot.eu-west-1.amazonaws.com"
    PORT = 8883
    CERT_FILE = "cert_rpipico/PiPico-certificate.pem.crt"
    KEY_FILE = "cert_rpipico/PiPico-private.pem.key"
    CA_FILE = "cert_rpipico/AmazonRootCA1.pem"

    # CERT_FILE = "certs_n/certificate.der.crt"
    # KEY_FILE = "certs_n/private.der.key"
    # CA_FILE = "certs_n/mazon_ca.der.crt"
    TOPIC = "raspberry"

    ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
    ssl_context.verify_mode = ssl.CERT_REQUIRED
    ssl_context.load_cert_chain(CERT_FILE, KEY_FILE)
    ssl_context.load_verify_locations(cafile=CA_FILE)

    # Configurar el cliente MQTT sin sl_params
    client = MQTTClient(
        client_id=CLIENT_ID, server=SERVER, port=PORT, keepalive=5000, ssl=ssl_context
    )

    client.connect()
    print("Conectado al servidor MQTT")


def publish(mensaje):
    client.publish(TOPIC, mensaje)
    print(f"Mensaje publicado: {mensaje}")


# Lectura sensores
def read_voltage_total():
    temp_voltage = vbatt_in.read_u16() * (CAL_REFERENCE_VOLTAGE / 65535.0)
    return temp_voltage * CAL_BATTERY_TOTAL


def read_voltage_lower():
    temp_voltage = vbatt1_in.read_u16() * (CAL_REFERENCE_VOLTAGE / 65535.0)
    return temp_voltage * CAL_BATTERY_LOWER


def read_current():
    temp_current = amps_in.read_u16() * (CAL_REFERENCE_VOLTAGE / 65535.0)
    return temp_current * CAL_CURRENT


def read_throttle():
    temp_throttle = throttle_in.read_u16() * (CAL_REFERENCE_VOLTAGE / 65535.0)
    if temp_throttle < 1:
        temp_throttle = 0
    elif temp_throttle > 4:
        temp_throttle = CAL_REFERENCE_VOLTAGE
    return int((temp_throttle / CAL_REFERENCE_VOLTAGE) * 100)


def read_t1():
    return thermistor_adc_to_celsius(temp1_in.read_u16())


def read_t2():
    return thermistor_adc_to_celsius(temp2_in.read_u16())


def thermistor_adc_to_celsius(adc_value):
    if adc_value >= 65535:
        adc_value = 65534  # Evitar la divisin por cero

    A = CAL_THERM_A
    B = CAL_THERM_B
    C = CAL_THERM_C
    FIXED_RESISTOR_VALUE = 10000
    thermistor_resistance = (FIXED_RESISTOR_VALUE * adc_value) / (65535 - adc_value)
    ln_resistance = math.log(thermistor_resistance)
    temperature = 1 / (A + (B * ln_resistance) + (C * ln_resistance**3))
    return temperature - 273.15


def read_wheel_speed():
    global wheelPoll, lastWheelSpeedPollTime
    temp_wheel_poll = wheelPoll
    wheelPoll = 0
    temp_last_wheel_poll_time = lastWheelSpeedPollTime
    lastWheelSpeedPollTime = utime.ticks_ms()

    if temp_last_wheel_poll_time == lastWheelSpeedPollTime:
        lastWheelSpeedPollTime += 1  # Evita la divisin por cero

    wheel_rpm = temp_wheel_poll / CAL_WHEEL_MAGNETS
    wheel_rpm = wheel_rpm / (
        (lastWheelSpeedPollTime - temp_last_wheel_poll_time) / 60000.0
    )

    if (lastWheelSpeedPollTime - temp_last_wheel_poll_time) == 0:
        wheel_rpm = 0  # Evita la divisin por cero

    return wheel_rpm


def read_motor_rpm():
    global motorPoll, lastMotorSpeedPollTime
    temp_motor_poll = motorPoll
    motorPoll = 0
    temp_last_motor_poll_time = lastMotorSpeedPollTime
    lastMotorSpeedPollTime = utime.ticks_ms()

    if temp_last_motor_poll_time == lastMotorSpeedPollTime:
        lastMotorSpeedPollTime += 1  # Evita la divisin por cero

    motor_revolutions = temp_motor_poll / CAL_MOTOR_MAGNETS
    motor_revolutions_per_min = motor_revolutions * 60.0
    time_diff_ms = lastMotorSpeedPollTime - temp_last_motor_poll_time
    time_diff_s = time_diff_ms / 1000.0

    if time_diff_s == 0:
        time_diff_s = 0.001  # Evita la divisin por cero

    motor_shaft_rpm = motor_revolutions_per_min / time_diff_s
    return motor_shaft_rpm


def calculate_gear_ratio():
    wheel_speed = read_wheel_speed()
    if wheel_speed == 0:
        return 0  # Evita la divisin por cero
    return read_motor_rpm() / wheel_speed


def leer_datos_imu():
    thr = read_throttle()
    bat_vol_tot = read_voltage_total()
    bat_vol_low = read_voltage_lower()
    c = read_current()
    temp1 = read_t1()
    temp2 = read_t2()
    wheel_speed = read_wheel_speed()
    motor_rpm = read_motor_rpm()
    g_ratio = calculate_gear_ratio()

    datos_sensores = {
        "thr": thr,
        "bat_vol_tot": bat_vol_tot,
        "bat_vol_low": bat_vol_low,
        "c": c,
        "temp1": temp1,
        "temp2": temp2,
        "wheel_speed": wheel_speed,
        "motor_rpm": motor_rpm,
        "g_ratio": g_ratio,
    }

    device_id = 1
    t = utime.time()
    t = round(t, 0)
    datos = {
        "device_id": device_id,
        "sample_time": t,
        "datos_imu": datos_sensores,
        "eventos": True,
        "timestamp": utc_isoformat(),
    }
    mensaje = json.dumps(datos)

    return mensaje


def utc_isoformat():
    # Obtener la hora actual en segundos desde el epoch
    now = utime.time()

    # Convertir a una estructura de tiempo UTC
    tm = utime.gmtime(now)

    # Formatear la fecha y la hora al estilo ISO 8601
    iso_format = "{}-{:02d}-{:02d}T{:02d}:{:02d}:{:02d}Z".format(
        tm[0], tm[1], tm[2], tm[3], tm[4], tm[5]
    )

    return iso_format


def main():
    global lastShortDataSendTime, loopCounter, client
    sync_time()
    connect()

    try:
        while True:
            mensaje = leer_datos_imu()
            publish(mensaje)
            utime.sleep(1)

    except KeyboardInterrupt:
        pass
    finally:
        client.disconnect()


if __name__ == "__main__":
    main()
