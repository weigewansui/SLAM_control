. setenv.sh
bot-procman-deputy &
DEPUTY=$!
bot­-procman­-sheriff ./config/procman­-maebot.cfg &
SHRIFF=$!

bot-lcm-tunnel 192.168.3.105
TUNNEL=$!

read KILL

kill $DEPUTY
kill $TUNNEL
kill $SHRIFF

