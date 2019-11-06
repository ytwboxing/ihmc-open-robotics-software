#!/bin/bash

OLDEST_LOG_AGE_HOURS=3

mkdir -p $HOME/.ihmc/logs/
find $HOME/.ihmc/logs/  \( -name "*.log" -o -name "*.json" \) -type f -mmin +$((60 * $OLDEST_LOG_AGE_HOURS)) -delete;
java -Djava.library.path=lib/ -cp ValkyrieController.jar us.ihmc.valkyrie.ValkyrieROS2LoadTestLink
