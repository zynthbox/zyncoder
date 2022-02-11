import logging
import time
from zyncoder import *
from zyncoder import lib_zyncoder_init

lib_zyncoder_init()
lib_zyncoder = get_lib_zyncoder()

last_zynswitch_index = lib_zyncoder.get_last_zynswitch_index()
num_zynpots = lib_zyncoder.get_num_zynpots()

while True:
    i = 0
    #Configure zynpots
    for i in range(num_zynpots):
        lib_zyncoder.setup_rangescale_zynpot(i, 0, 100, 50, 0);

    for i in range(last_zynswitch_index + 1):
        dtus = lib_zyncoder.get_zynswitch(i, 2000000)
        if dtus > 0:
            logging.error("state: {} {} {}".format(i, dtus, 2000000))

    for i in range(num_zynpots):
        if lib_zyncoder.get_value_flag_zynpot(i):
            logging.error("PT-{} = {}".format(i, lib_zyncoder.get_value_zynpot(i)))
