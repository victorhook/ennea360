# ennea360


## Ardupilot configuration
The following parameters has to be set for each sensor, so 9 in total
```
RNGFND1_TYPE 10
RNGFND1_SCALING 1
RNGFND1_MIN_CM 5
RNGFND1_MAX_CM 120
RNGFND1_GNDCLEAR 10
RNGFND1_POS_X 5
RNGFND1_POS_Y 0
RNGFND1_POS_Z 0
RNGFND1_ORIENT 0
```

### Obstacle avoidance
```
AVOID_ENABLE 7 # All input sources
AVOID_DIST_MAX 0.5 # 0.5 meter
AVOID_MARGIN 0.5 # 0.5 meter
AVOID_BEHAVE 0 # Slide
AVOID_BACKUP_SPD 0.5
AVOID_ALT_MIN 0
AVOID_ACCEL_MAX 3
AVOID_BACKUP_DZ 0.1
AVOID_BACKZ_SPD 0.75
PRX1_TYPE 4 # Rangefinder
```