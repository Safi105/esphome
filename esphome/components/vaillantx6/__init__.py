
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import sensor, binary_sensor, uart
from esphome.const import CONF_ID, CONF_NAME, CONF_UPDATE_INTERVAL

DEPENDENCIES = ['uart']

Vaillantx6Sensor = cg.global_ns.class_('Vaillantx6Sensor', cg.PollingComponent, uart.UARTDevice)

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(Vaillantx6Sensor),
    cv.Optional(CONF_UPDATE_INTERVAL, default="60s"): cv.positive_time_period_milliseconds,
}).extend(uart.UART_DEVICE_SCHEMA).extend(sensor.sensor_schema()).extend(binary_sensor.binary_sensor_schema())

def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    cg.add(var.set_update_interval(config[CONF_UPDATE_INTERVAL]))
    cg.add(var.set_uart_parent(cg.get_variable(config[CONF_UART_ID])))