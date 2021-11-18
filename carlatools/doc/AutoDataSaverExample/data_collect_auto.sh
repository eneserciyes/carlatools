export TEAM_AGENT=/home/enes/avg/carlatools/carlatools/doc/AutoDataSaverAgentExample.py
export TEAM_CONFIG=/home/enes/avg/carlatools/carlatools/doc/AutoDataSaverAgent.yaml
export CHALLENGE_TRACK_CODENAME=SENSORS
export PORT=2000
export TM_PORT=8000
export DEBUG=0

python3 $LEADERBOARD_ROOT/leaderboard/leaderboard_evaluator.py --port=${PORT} --trafficManagerPort=${TM_PORT} --track=SENSORS --scenarios=${LEADERBOARD_ROOT}/data/all_towns_traffic_scenarios_public.json --agent=${TEAM_AGENT} --agent-config=${TEAM_CONFIG} --routes=${ROUTES}