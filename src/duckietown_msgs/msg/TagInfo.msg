Header header
int32 id

# (StreetName, TrafficSign, Localization, Vehicle…)
uint8 tag_type

uint8 S_NAME=0
uint8 SIGN=1	
uint8 LIGHT=2
uint8 LOCALIZE=3
uint8 VEHICLE=4

string street_name

uint8 traffic_sign_type
# (12 possible traffic sign types)

uint8 STOP=5
uint8 YIELD=6
uint8 NO_RIGHT_TURN=7
uint8 NO_LEFT_TURN=8
uint8 ONEWAY_RIGHT=9
uint8 ONEWAY_LEFT=10
uint8 FOUR_WAY=11
uint8 RIGHT_T_INTERSECT=12
uint8 LEFT_T_INTERSECT=13
uint8 T_INTERSECTION=14
uint8 DO_NOT_ENTER=15
uint8 PEDESTRIAN=16
uint8 T_LIGHT_AHEAD=17
uint8 DUCK_CROSSING=18
uint8 PARKING=19

string vehicle_name

# Just added a single number for location. Probably want to use Vector2D.msg, but I get errors when I try to add it.
float32 location
