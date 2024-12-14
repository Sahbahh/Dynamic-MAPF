# Dynamic-MAPF

python3 run_experiments.py --instance instances/exp0.txt --solver Independent

python main.py d_map4.json --algorithm a_star

python main.py d_map4.json --algorithm Prioritized

python3 main.py d_map4.json --algorithm CBS

python3 main.py d_map4.json --algorithm "CBS Disjoint"

To run all the algorithms:
python3 run_all_algorithms.py d_map4.json