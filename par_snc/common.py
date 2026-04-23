HAZARD_IDS = {
    "unknown": 0,
    "explosive": 1,
    "flammable_gas": 2,
    "non_flammable_gas": 3,
    "dangerous_when_wet": 4,
    "flammable_solid": 5,
    "spontaneously_combustible": 6,
    "oxidizer": 7,
    "organic_peroxide": 8,
    "inhalation_hazard": 9,
    "poison": 10,
    "radioactive": 11,
    "corrosive": 12,
}

OBJECT_NAME_TO_HAZARD_KEY = {
    "explosive": "explosive",
    "flammable_gas": "flammable_gas",
    "non_flammable_gas": "non_flammable_gas",
    "dangerous_when_wet": "dangerous_when_wet",
    "flammable_solid": "flammable_solid",
    "spontaneously_combustible": "spontaneously_combustible",
    "oxidizer": "oxidizer",
    "organic_peroxide": "organic_peroxide",
    "inhalation_hazard": "inhalation_hazard",
    "poison": "poison",
    "radioactive": "radioactive",
    "corrosive": "corrosive",
}

MAP_FRAME = "map"
BASE_FRAME = "base_link"

STATUS_TOPIC = "/snc_status"
HAZARD_TOPIC = "/hazards"
PATH_EXPLORE_TOPIC = "/path_explore"
PATH_RETURN_TOPIC = "/path_return"

TRIGGER_START_TOPIC = "/trigger_start"
TRIGGER_HOME_TOPIC = "/trigger_home"
TRIGGER_TELEOP_TOPIC = "/trigger_teleop"

HAZARD_INSPECT_TOPIC = "/hazard_inspect_request"