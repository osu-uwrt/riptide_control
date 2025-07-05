#TODO: sort these
ERROR_PATIENCE = 1.0

#the number of frames the escs can be powered off before clearing acculators
ESC_POWER_STOP_TOLERANCE = 2
ESC_POWER_TIMEOUT = 2

PARAMETER_SCALE = 1000000

THRUSTER_SOLVER_WEIGHT_MATRIX_TOPIC = "controller/solver_weights"

#paramters that cannot be pulled from yaml but still need setting
SPECIAL_PARAMETERS = ["talos_wrenchmat"]

ACTIVE_PARAMETERS_MASK = "controller__active_force_mask"

FF_PUBLISH_PARAM = "disable_native_ff"
FF_TOPIC_NAME = "controller/FF_body_force"
AUTOTUNE_REINIT_TOPIC_NAME = "controller/re_init_accumulators"
WEIGHTS_FORCE_UPDATE_PERIOD = 1

ORIN_AUTOTUNE_DIR = "/bin"

AUTOFF_INIT_TOLERANCE = .01 #seeing rounding errors lol

G = 9.8067

RELOAD_TIME = 2 # seconds

