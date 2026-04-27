import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import number
from . import TimotionDeskControllerComponent, CONF_TIMOTION_DESK_CONTROLLER_ID

DEPENDENCIES = ["esp32", "timotion_desk_controller"]

CONFIG_SCHEMA = number.number_schema(number.Number).extend(
    {
        cv.GenerateID(CONF_TIMOTION_DESK_CONTROLLER_ID): cv.use_id(TimotionDeskControllerComponent),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_TIMOTION_DESK_CONTROLLER_ID])
    await number.register_number(
        hub,
        config,
        min_value=65,
        max_value=130,
        step=1,
    )
