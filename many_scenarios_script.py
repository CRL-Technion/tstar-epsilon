import toml
import os
import random
import time


start = time.time()
# Map size = 15 X 15
# Edit toml file
size = 15
# sizes = [15, 30, 50, 100]
sizes = [15]
orientations = 8
speeds = 2

# Outputs file
# map_number = '31'
windx = '_windx_0_'
windy = 'windy_0_'
maps = [str(i) for i in range(31, 41)]
# folder = 'monte_carlo_with_wind'
folder = 'monte_carlo_for_lower_bounds'
# min_speeds = ['0.15', '0.2', '0.4', '0.5', '0.6', '0.8', '0.9', '0.95']
# min_speeds = ['0.2']
# cover_by_obstacles = '25%'
# logs_file_mame = 'results_u_0_5_vmin_0_5_map' + map_number + cover_by_obstacles + '.txt'
# logs_file_mame = 'results_u_1_vmin_0_5_map' + map_number + windx + windy + '_euclideanDistance.txt'
# logs_file_mame = 'results_u_1_vmin_0_5_map' + map_number + windx + windy + '_with_number_of_states_in_the_solution.txt'
# logs_file_mame = 'results_u_1_vmin_0_5_map20_windx_02__windy_0__with_number_of_states_in_the_solution.txt'
# logs_file_mame = 'results_u_1_vmin_0.5' + '_map' + map_number + windx + windy + '.txt'
# run_command = './run.sh >> ' + logs_file_mame
# logs_file_mame = folder + '/results_u_1_vmin_0.5' + '_map' + map_number + windx + windy + '.txt'
logs_file_mame = folder + '/results_u_1_vmin_0.5' + '_all_maps_with_euclidean_distance.txt'

iterations = 50

eps_values = ['1.1', '1.3', '1.5', '2', '4']
# for size in sizes:
for iter in range(iterations):
    # logs_file_mame = folder + '/results_u_1_vmin_0.6' + '_map' + map_number + windx + windy + '.txt'
    for map_number in maps:
# for map_number in maps:
#     # logs_file_mame = folder + '/results_u_1_vmin_0.6' + '_map' + map_number + windx + windy + '.txt'
#     for iter in range(iterations):
        # f = open(logs_file_mame, 'a')
        # f.write('Start of scenario\n')
        # f.close()

        # Choose the problem parameters
        # All the 0's and last rows/columns are occupied by obstacles
        row1 = random.randint(1, size - 2)
        col1 = random.randint(1, size - 2)
        theta1 = random.randint(0, orientations - 1)
        # v1 = random.randint(0, speeds - 1)
        v1 = 0

        # All the 0's and last rows/columns are occupied by obstacles
        row2 = random.randint(1, size - 2)
        col2 = random.randint(1, size - 2)
        theta2 = random.randint(0, orientations - 1)
        v2 = v1

        if iter % 10 == 0:
            print(f'\n\niteration: {iter}\n\n')
        print(f'map: {map_number}', end=' ')
        print(f'start = [{row1}, {col1}, {theta1}, {v1}]', end=' ')
        print(f'goal = [{row2}, {col2}, {theta2}, {v2}]')

        # for min_v in min_speeds:
        #     print(f'Minimum speed: {min_v}')
        #     folder = 'u_1_vmin_' + min_v + windx + windy
        #     os.system('cp  python/calculated_ocps/' + folder + '/ocps.txt python/ocps.txt')
        #     logs_file_mame = 'results_u_1_vmin_' + min_v + '_map' + map_number + windx + windy + '.txt'

        f = open(logs_file_mame, 'a')
        f.write('Start of scenario\n')
        f.close()

        # Define the problem parameters
        data = toml.load('python/config.toml')
        data['map'] = "../maps/map" + map_number + ".txt"
        data['start'] = [row1, col1, theta1, v1]
        data['goal'] = [row2, col2, theta2, v2]

        # data['vMin'] = float(min_v)

        # T* run
        # Indicator for start of T*
        f = open(logs_file_mame, 'a')
        f.write('T* start\n')
        f.close()

        data['algorithm'] = 'tstar'
        data['g_func'] = 'varspeed'
        run_command = './run.sh >> ' + logs_file_mame

        f = open("python/config.toml", 'w')
        toml.dump(data, f)
        f.close()
        os.system(run_command)

        # Indicator for end of T*
        f = open(logs_file_mame, 'a')
        f.write('T* end\n')
        f.close()

        # T*-epsilon run
        # Indicator for start of T*-eps
        f = open(logs_file_mame, 'a')
        f.write('T*-eps start\n')
        f.close()

        data['algorithm'] = 'tstar-epsilon'
        data['g_func'] = 'dubins'

        for eps in eps_values:
            data['speed'] = v1
            data['epsilon'] = float(eps)
            f = open("python/config.toml", 'w')
            toml.dump(data, f)
            f.close()
            os.system(run_command)

        # Indicator for end of T*-eps
        f = open(logs_file_mame, 'a')
        f.write('T*-eps end\n')
        f.close()

        f = open(logs_file_mame, 'a')
        f.write('End of scenario\n')
        f.close()
            # os.system('./run.sh >> results_u_0_5_vmin_0_5_37%_obstacles.txt')
end = time.time()
print(f'Running time = {end - start} seconds')
# starts = [[6, 19, 0, 0], [22, 19, 3, 0], [24, 27, 7, 0], [7, 5, 2, 0], [6, 9, 6, 0], [25, 9, 7, 0], [7, 27, 1, 0], [22, 5, 7, 0], [26, 20, 2, 0], [14, 5, 3, 0]]
# goals = [[4, 3, 4, 0], [25, 8, 0, 0], [11, 2, 1, 0], [24, 23, 4, 0], [23, 8, 3, 0], [12, 28, 5, 0], [8, 22, 1, 0], [9, 9, 1, 0], [23, 4, 0, 0], [17, 20, 2, 0], [3, 11, 0, 0]]
#
# for iter in range(len(starts)):
#     f = open(logs_file_mame, 'a')
#     f.write('Start of scenario\n')
#     f.close()
#
#     # Choose the problem parameters
#     # All the 0's and last rows/columns are occupied by obstacles
#     # row1 = random.randint(1, size - 2)
#     # col1 = random.randint(1, size - 2)
#     # theta1 = random.randint(0, orientations - 1)
#     # # v1 = random.randint(0, speeds - 1)
#     # v1 = 0
#     #
#     # # All the 0's and last rows/columns are occupied by obstacles
#     # row2 = random.randint(1, size - 2)
#     # col2 = random.randint(1, size - 2)
#     # theta2 = random.randint(0, orientations - 1)
#     # v2 = v1
#
#     if iter % 10 == 0:
#         print(f'\n\niteration: {iter}\n\n')
#     # print(f'start = [{row1}, {col1}, {theta1}, {v1}]', end=' ')
#     # print(f'goal = [{row2}, {col2}, {theta2}, {v2}]')
#     print(f'start = {starts[iter]}', end=' ')
#     print(f'goal = {goals[iter]}')
#
#     # Define the problem parameters
#     data = toml.load('python/config.toml')
#     data['map'] = "../maps/map" + map_number + ".txt"
#     data['start'] = starts[iter]
#     data['goal'] = goals[iter]
#
#     # T* run
#     # Indicator for start of T*
#     f = open(logs_file_mame, 'a')
#     f.write('T* start\n')
#     f.close()
#
#     data['algorithm'] = 'tstar'
#     data['g_func'] = 'varspeed'
#     # run_command = './run.sh >> ' + logs_file_mame
#
#     f = open("python/config.toml", 'w')
#     toml.dump(data, f)
#     f.close()
#     os.system(run_command)
#
#     # Indicator for end of T*
#     f = open(logs_file_mame, 'a')
#     f.write('T* end\n')
#     f.close()
#
#     # T*-epsilon run
#     # Indicator for start of T*-eps
#     f = open(logs_file_mame, 'a')
#     f.write('T*-eps start\n')
#     f.close()
#
#     data['algorithm'] = 'tstar-epsilon'
#     data['g_func'] = 'dubins'
#
#     for eps in eps_values:
#         data['speed'] = 0
#         data['epsilon'] = float(eps)
#         f = open("python/config.toml", 'w')
#         toml.dump(data, f)
#         f.close()
#         os.system(run_command)
#
#     # Indicator for end of T*-eps
#     f = open(logs_file_mame, 'a')
#     f.write('T*-eps end\n')
#     f.close()
#
#     f = open(logs_file_mame, 'a')
#     f.write('End of scenario\n')
#     f.close()
#     # os.system('./run.sh >> results_u_0_5_vmin_0_5_37%_obstacles.txt')
