. setenv.sh
bot-procman-deputy &
DEPUTY=$!
bot-lcm-tunnel
TUNNEL=$!

read KILL

kill $DEPUTY
kill $TUNNEL

