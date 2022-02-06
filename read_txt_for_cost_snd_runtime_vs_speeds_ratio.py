import matplotlib.pyplot as plt
import numpy as np
import numpy.linalg as la
import statistics as stat
from prettytable import PrettyTable
from decimal import *
import matplotlib.ticker as mtick
import matplotlib as mpl
from scipy.stats import sem
from matplotlib.font_manager import FontProperties
mpl.rc('text', usetex = True)
mpl.rc('font', family = 'serif', size=20)
mpl.rcParams['text.latex.preamble']=[r"\usepackage{amsmath}"]

def clear_string(str):
    out_str = ''
    for ch in str:
        if ch.isdigit() or ch == '.':
            out_str += ch
    return out_str


def calc_lower_bound_quality(filename):
    file = open(filename, 'r')
    lines = file.readlines()

    wind = list()
    count = 0
    lower_bound_lines = list()
    # Get the lines with lower bound to time-optimal ratio
    for line in lines:
        if line.split(" ")[0] == 'Lower':
            lower_bound_lines.append(line.split())
            count += 1
        if line.split(" ")[0] == 'wind':
            wind.append(float(clear_string(line.split(" ")[2])))
            wind.append(float(clear_string(line.split(" ")[3])))


    bounds = list()
    for i in range(len(lower_bound_lines)):
        bound = lower_bound_lines[i][6].replace("%", "")
        bounds.append(float(bound))


    avg = sum(bounds) / len(bounds)
    print(f'Wind {wind} modulus = {la.norm(wind)}')
    return wind, avg


def get_wind_and_average(filename):
    file = open(filename, 'r')
    lines = file.readlines()

    wind = list()
    count = 0
    lower_bound_lines = list()
    # Get the lines with lower bound to time-optimal ratio
    for line in lines:
        if line.split(" ")[0] == 'Lower':
            lower_bound_lines.append(line.split())
            count += 1
        if line.split(" ")[0] == 'wind':
            wind_x = (float(clear_string(line.split(" ")[2])))
            wind_y = (float(clear_string(line.split(" ")[3])))
            wind.append(la.norm([wind_x, wind_y]))

    bounds = list()
    for i in range(len(lower_bound_lines)):
        bound = lower_bound_lines[i][6].replace("%", "")
        bounds.append(float(bound))

    # avg = sum(bounds) / len(bounds)
    # print(f'Wind {wind} modulus = {la.norm(wind)}')
    return wind, bounds


def get_unique_transitions(filename):
    file = open(filename, 'r')
    lines = file.readlines()

    states = list()
    count = 0
    unique_transitions = list()
    for line in lines:
        if line.split(" ")[0] == 'Unique':
            val = line.split(" ")[-1]
            unique_transitions.append(line.split(" ")[-1])
        if line.split(" ")[0] == 'Number':
            states.append(line.split(" ")[-1])

    states_re = [int(x) for x in states]
    unique_re = [int(x) for x in unique_transitions]

    return unique_re, states_re


def get_relative(line: str):
    # get line and return the relative transition as list
    lst = line.split('[')
    lst = lst[-1].split(']')
    lst = lst[0].split(',')
    return lst


def check_size_fit(a: list, b: str):
    return len(a) == int(b)


def get_tstar_vals(start_line, lines):
    number_of_lines = 0
    cost = -1
    runtime = -1
    for i, loc_lines in enumerate(lines, start=start_line):
        line = lines[i]
        if 'Cost to path:' in lines[i]:
            cost = float(line.split(' ')[-1])
        elif 'planing time:' in lines[i]:
            runtime = float(line.split(' ')[-1])
        elif lines[i] == 'T* end\n':
            return cost, runtime, number_of_lines
        number_of_lines += 1


def get_eps_vals(start_line, lines, single_opt):
    number_of_lines = 0
    costs_count = 0
    trans_count = 0
    dubins_time = 0
    cost = None
    runtime = None
    transitions = None
    for i, loc_lines in enumerate(lines, start=start_line):
        line = lines[i]
        if line == 'No solution\n':
            return None, None, None
        elif 'Cost to path:' in line and costs_count == 1:
            cost = float(line.split(" ")[-1])
        elif 'Cost to path:' in line:
            costs_count += 1
        elif 'duration of calc Dubibs paths' in line:
            dubins_time += float(line.split(' ')[-1])
        elif 'Time to calculate the first step' in line:
            dubins_time += float(line.split(' ')[-2])
        # elif 'Time-optimal translations calculate time:' in line:
        #     opt_time = float(line.split(' ')[-2])
        #     search_time = float(lines[i + 1].split(' ')[-2])
        # elif 'T*-' and 'time:' in line:
        #     search_time = float(line.split(' ')[-2])
        elif 'Number of translations:' in line and trans_count == 1:
            transitions = int(line.split(' ')[-1])
        elif 'Total runtime of T*-epsilon:' in line:
            runtime = float(line.split(' ')[-1])
        elif 'Number of translations:' in line:
            trans_count += 1
        elif 'plot time:' in line:
            runtime += transitions * single_opt + dubins_time
            return cost, runtime, number_of_lines
        number_of_lines += 1


def monte_carlo_read(filename, tstar_opt_time):
    getcontext().prec = 3
    file = open(filename, 'r')
    lines = file.readlines()
    line_num = 0

    total_transitions = 512
    single_opt = tstar_opt_time / total_transitions

    # [eps = 1.1, eps = 1.3, eps = 1.5, eps = 2, eps = 4]
    eps_costs = [None] * 5
    eps_runtime = [None] * 5
    eps_transitions = [-1] * 5
    eps_dubins_times = [-1] * 5
    eps_opt_times = [-1] * 5
    eps_search_times = [-1] * 5
    tstar_cost = -1
    tstar_loc_search_time = -1

    tstar_search_time = []
    costs = [[], [], [], [], []]
    runtime = [[], [], [], [], []]
    dubins_time = [[], [], [], [], []]
    optimizations_time = [[], [], [], [], []]
    tstar_eps_time = [[], [], [], [], []]
    transitions = [[], [], [], [], []]

    while line_num < len(lines):
        if lines[line_num] == 'T* start\n':
            tstar_cost, tstar_loc_search_time, foward_lines = get_tstar_vals(line_num, lines)
            line_num += foward_lines
            if tstar_cost == -1:  # No solution for T*, no need to check T*-eps solutions
                # Run until instance end
                while lines[line_num] != 'End of scenario\n':
                    line_num += 1
                continue
            else:
                tstar_loc_search_time += tstar_opt_time
                tstar_search_time.append(tstar_loc_search_time)
        elif lines[line_num] == 'T*-eps start\n':
            # Collect data from T*-epsilon runs
            # TODO foward lines after get_eps_vals
            while lines[line_num] != 'T*-eps end\n':
                move = 0
                if lines[line_num] == 'epsilon = 1.1\n':
                    eps_costs[0], eps_runtime[0], move = get_eps_vals(line_num, lines, single_opt)
                    if eps_costs[0] == None:
                        while lines[line_num] != 'T*-eps end\n':
                            line_num += 1
                        continue
                elif lines[line_num] == 'epsilon = 1.3\n':
                    eps_costs[1], eps_runtime[1], move = get_eps_vals(line_num, lines, single_opt)
                elif lines[line_num] == 'epsilon = 1.5\n':
                    eps_costs[2], eps_runtime[2], move = get_eps_vals(line_num, lines, single_opt)
                elif lines[line_num] == 'epsilon = 2\n':
                    eps_costs[3], eps_runtime[3], move = get_eps_vals(line_num, lines, single_opt)
                elif lines[line_num] == 'epsilon = 4\n':
                    eps_costs[4], eps_runtime[4], move = get_eps_vals(line_num, lines, single_opt)

                line_num += move + 1

            if None not in eps_costs:
                eps_costs = [cost / tstar_cost for cost in eps_costs]
                eps_runtime = [runtime / tstar_loc_search_time for runtime in eps_runtime]
                for i in range(len(eps_costs)):
                    costs[i].append(eps_costs[i])
                    runtime[i].append(eps_runtime[i])
                    # transitions[i].append(eps_transitions[i])
                    # dubins_time[i].append(eps_dubins_times[i])
                    # eps_opt_times[i] = single_opt * eps_transitions[i]
                    # optimizations_time[i].append(eps_opt_times[i])
                    # tstar_eps_time[i].append(eps_search_times[i])
                eps_costs = [None] * 5
                eps_runtime = [None] * 5
                tstar_cost = -1

        line_num += 1

    avg_cost = [np.array(cost).mean() for cost in costs]
    avg_cost = np.array(avg_cost)
    std_cost = [sem(cost) for cost in costs]
    std_cost = np.array(std_cost)

    avg_runtime = [np.array(run).mean() for run in runtime]
    avg_runtime = np.array(avg_runtime)
    std_runtime = [sem(run) for run in runtime]
    std_runtime = np.array(std_runtime)

    print(f'Number of examples in {filename} = {len(costs[0])}')

    return avg_cost, std_cost, avg_runtime, std_runtime
    # avg_cost = [Decimal(sum(i)) / Decimal(len(i)) for i in costs]
    # std_costs = [Decimal(stat.stdev(i)) / Decimal(1) for i in costs]
    # costs_median = [Decimal((np.median(i))) / Decimal(1) for i in costs]
    #
    # total_runtime = [[(dubins_time[i][j] + optimizations_time[i][j] + tstar_eps_time[i][j]) for j in range(len(dubins_time[0]))] for i in range(len(dubins_time))]
    # avg_runtime = [Decimal(sum(i)) / Decimal(len(i)) for i in total_runtime]
    # # relative_dubins = [sum(i) / len(i) / j for i in dubins_time for j in avg_runtime]
    # # relative_opt = [sum(i) / len(i) / j for i in optimizations_time for j in avg_runtime]
    # # relative_search = [sum(i) / len(i) / j for i in tstar_eps_time for j in avg_runtime]
    #
    # relative_runtime = [Decimal((sum(i) / len(i)) / tstar_opt_time) * Decimal(100) for i in total_runtime]
    # median_runtime = [Decimal((np.median(i) / tstar_opt_time)) * Decimal(100) for i in total_runtime]
    #
    # relative_transitions = [Decimal((sum(i) / len(i)) / total_transitions) * Decimal(100) for i in transitions]
    # median_transitions = [Decimal((np.median(i) / total_transitions)) * Decimal(100) for i in transitions]
    #
    # avg_cost.insert(0, 'average cost')
    # std_costs.insert(0, 'cost std')
    # costs_median.insert(0, 'cost median')
    # relative_runtime.insert(0, 'average % runtime')
    # median_runtime.insert(0, 'median % runtime')
    # relative_transitions.insert(0, 'average % transitions')
    # median_transitions.insert(0, 'median % transitions')
    #
    # print(f'\nSummery of {filename} with {len(costs[0])} experiments:\n')
    #
    # tbl = PrettyTable(['', 'eps = 1.1', 'eps = 1.3', 'eps = 1.5', 'eps = 2', 'eps = 4'], digits=2)
    # tbl.add_row(avg_cost)
    # tbl.add_row(std_costs)
    # tbl.add_row(costs_median)
    # tbl.add_row(relative_runtime)
    # tbl.add_row(median_runtime)
    # tbl.add_row(relative_transitions)
    # tbl.add_row(median_transitions)
    # print(tbl)


def mutual_transitions(filename):
    file = open(filename, 'r')
    lines = file.readlines()

    states_in_tstar = list()
    unique_transitions_in_tstar = list()
    percentage_of_dubins_in_tstar = list()
    temp_dubins_transitions = []
    temp_tstar_transitions = []
    tstar_unique_transitions = 0
    dubins_unique_transitions = 0


    s_scenario = False
    e_scenario = False
    s_tstar = False
    e_tstar = False
    tstat_sol = False
    s_dubins = False
    e_dubins = False
    dubins_sol = False

    tstar_num_of_states = 0

    for line in lines:
        if line == 'Start of scenario\n':
            s_scenario = True
        elif line == 'varspeed start\n':
            s_tstar = True
        elif s_tstar and line == 'The path from goal to start:\n':  # In tstar and there is solution
            tstat_sol = True
        elif line == 'varspeed end\n':
            s_tstar = False
            e_tstar = True
        elif tstat_sol and line == 'dubins start\n':
            s_dubins = True
        elif s_dubins and tstat_sol and line == 'The path from goal to start:\n':  # If tstar has no solution there is no point to check dubins
            dubins_sol = True
        elif line == 'dubins end\n':
            s_dubins = False
            e_dubins = True
        elif line == 'End of scenario\n':
            e_scenario = True

        in_tstar = s_tstar and not e_tstar
        in_dubis = s_dubins and not e_dubins

        # Tstar solution collect data

        if s_scenario and in_tstar and tstat_sol:
            # Build unique transitions list
            if '>' in line:
                transition = get_relative(line)
                if transition not in temp_tstar_transitions:
                    temp_tstar_transitions.append(transition)
            elif line.split(" ")[0] == 'Unique':
                tstar_unique_transitions = line.split(" ")[-1]
            elif line.split(" ")[0] == 'Number':
                tstar_num_of_states = line.split(" ")[-1]
        elif s_scenario and in_dubis and dubins_sol:
            # Build unique transitions list
            if '>' in line:
                transition = get_relative(line)
                if transition not in temp_dubins_transitions:
                    temp_dubins_transitions.append(transition)
            elif line.split(" ")[0] == 'Unique':
                dubins_unique_transitions = line.split(" ")[-1]
            elif line.split(" ")[0] == 'Number':
                dubins_num_of_states = line.split(" ")[-1]


        if e_scenario:
            # Add here the actions for end of scenario
            mutual = 0
            if dubins_sol:  # Means both tstar and Dubins have solution
                # if not check_size_fit(temp_tstar_transitions, tstar_unique_transitions):
                #     print("tstar size do not match")
                # if not check_size_fit(temp_dubins_transitions, dubins_unique_transitions):
                #     print("dubins size do not match")
                states_in_tstar.append(tstar_num_of_states)
                unique_transitions_in_tstar.append(tstar_unique_transitions)
                for val in temp_dubins_transitions:
                    if val in temp_tstar_transitions:
                        mutual += 1
                percentage_of_dubins_in_tstar.append(mutual / len(temp_tstar_transitions) * 100)

            tstar_unique_transitions = 0
            dubins_unique_transitions = 0
            dubins_num_of_states = 0
            tstar_num_of_states = 0
            temp_dubins_transitions = []
            temp_tstar_transitions = []
            s_scenario = s_tstar = e_tstar = s_dubins = e_dubins = tstat_sol = dubins_sol = e_scenario = False

    states_re = [int(x) for x in states_in_tstar]
    unique_re = [int(x) for x in unique_transitions_in_tstar]
    percentage_re = [float(x) for x in percentage_of_dubins_in_tstar]

    return states_re, unique_re, percentage_re

# names = ['eps_1_1_wind_01_0.txt', 'eps_1_1_wind_01_01.txt', 'eps_1_1_wind_02_0.txt', 'eps_1_1_wind_02_01.txt',
#          'eps_1_1_wind_02_02.txt', 'eps_1_1_wind_035_0.txt', 'eps_1_1_wind_03_02.txt', 'eps_1_1_wind_04_01.txt',
#          'eps_1_1_wind_04_02.txt']
# winds = list()
# lower = list()
#
# for name in names:
#     wind, avg = calc_lower_bound_quality(name)
#     winds.append(la.norm(wind))
#     lower.append(avg)


# winds2, lower_bounds2 = get_wind_and_average('1+wind_mudulus_lower_bound.txt')
#
# wind3, lower_bounds3 = get_wind_and_average('groundSpeedLowerBound.txt')
#
# wind4, lower_bounds4 = get_wind_and_average('headingGroundSpeedLowerBound.txt')
#
# wind5, lower_bounds5 = get_wind_and_average('sqrt_1+wind_sqr_lower_bound.txt')

# plt.plot(winds, lower, label=r'$\frac{Low\,speed\,Dubins}{ground\,speed}$')

# unique25, states25 = get_unique_transitions('results_u_0_5_vmin_0_5_25%_obstacles.txt')
# unique33, states33 = get_unique_transitions('results_u_0_5_vmin_0_5_33%_obstacles.txt')
# unique37, states37 = get_unique_transitions('results_u_0_5_vmin_0_5_37%_obstacles.txt')
#
#
# plt.scatter(states25, unique25)
# plt.scatter(states33, unique33)
# plt.scatter(states37, unique37)
# plt.ylim([0, 68])

# states25, unique25, percentage25 = mutual_transitions('results_u_0_5_vmin_0_5_map15_25%.txt')
# states33, unique33, percentage33 = mutual_transitions('results_u_0_5_vmin_0_5_map16_33%.txt')
# states37, unique37, percentage37 = mutual_transitions('results_u_0_5_vmin_0_5_map17_37%.txt')
#
# a = 1
#
# plt.scatter(states25, unique25)
# plt.scatter(states33, unique33)
# plt.scatter(states37, unique37)
# plt.ylim([0, 68])
# plt.show()
#
# plt.scatter(states25, percentage25)
# plt.scatter(states33, percentage33)
# plt.scatter(states37, percentage37)
# plt.ylim([0, 100])
# plt.show()
# plt.plot(winds2, lower_bounds2, label=r'$\frac{Low\,speed\,Dubins}{1 + |Wind|}$')
#
# plt.plot(wind3, lower_bounds3, label=r'$\frac{Low\,speed\,Dubins}{ground\,speed\,(direction)}$')
#
#
# plt.plot(wind5, lower_bounds5, label=r'$\frac{Low\,speed\,Dubins}{1 + Wind}$')

# plt.plot(wind4, lower_bounds4, label=r'$\frac{Low\,speed\,Dubins}{ground\,speed\,(heading)}$')

# plt.xlabel(r"$\vert Wind \vert$")
# plt.ylabel(r'$average(\frac{Lower\,bound}{Time-optimal}$%)')
# plt.title('Lower bound quality vs. wind modulus')
# plt.legend()
# plt.grid()
# plt.savefig('Lower bound quality.svg')
# plt.show()

# Duration of computing all transitions
# u_max = 1, v_min = 0.5
windx_02_windy_0_opt = 384.6
windx_04_windy_01_opt = 576.0

windx_02_windy_0_vmin_01_opt = 439.269
windx_02_windy_0_vmin_02_opt = 458.933
windx_02_windy_0_vmin_03_opt = 487.923
windx_02_windy_0_vmin_04_opt = 515.228
windx_02_windy_0_vmin_05_opt = 384.6
windx_02_windy_0_vmin_06_opt = 412.059
windx_02_windy_0_vmin_075_opt = 397.868
windx_02_windy_0_vmin_08_opt = 378.885
windx_02_windy_0_vmin_09_opt = 373.953

ocps_time = [windx_02_windy_0_vmin_01_opt, windx_02_windy_0_vmin_02_opt, windx_02_windy_0_vmin_03_opt,
             windx_02_windy_0_vmin_04_opt, windx_02_windy_0_vmin_05_opt, windx_02_windy_0_vmin_06_opt,
             windx_02_windy_0_vmin_075_opt, windx_02_windy_0_vmin_08_opt, windx_02_windy_0_vmin_09_opt]

# monte_carlo_read('results_u_1_vmin_0_5_map6_windx_02__windy_0_.txt', windx_02_windy_0_opt)
#
# monte_carlo_read('results_u_1_vmin_0_5_map6_windx_04__windy_01_.txt', windx_04_windy_01_opt)

standard_error_const = 1.96
speeds_ratio = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.75, 0.8, 0.9]
eps_cost_means = [[], [], [], [], []]
eps_cost_std = [[], [], [], [], []]
eps_runtime_mean = [[], [], [], [], []]
eps_runtime_std = [[], [], [], [], []]

for i, val in enumerate(ocps_time):
    name_of_file = 'results_u_1_vmin_' + str(speeds_ratio[i]) + '_map21_windx_0.2_windy_0.txt'
    cmean, cstd, rmean, rstd = monte_carlo_read(name_of_file, val)
    for j in range(len(eps_cost_means)):
        eps_cost_means[j].append(cmean[j])
        eps_cost_std[j].append(cstd[j])
        eps_runtime_mean[j].append(rmean[j])
        eps_runtime_std[j].append(rstd[j])

    a = 1

eps_cost_means = np.array(eps_cost_means)
eps_cost_std = np.array(eps_cost_std)
eps_runtime_mean = np.array(eps_runtime_mean)
eps_runtime_std = np.array(eps_runtime_std)

labels = [r'$\boldsymbol{\varepsilon}=0.1$', r'$\boldsymbol{\varepsilon}=0.3$', r'$\boldsymbol{\varepsilon}=0.5$',
          r'$\boldsymbol{\varepsilon}=1$', r'$\boldsymbol{\varepsilon}=3$']

fig = plt.figure(1, (7, 4))
ax = fig.add_subplot(1, 1, 1)

for i in range(len(eps_cost_means)):
    # fig = plt.figure(1, (7, 4))
    # ax = fig.add_subplot(1, 1, 1)
    # ax.plot(speeds_ratio, eps_runtime_mean[i], label=labels[i])
    # l = list(map(lambda val: 0 if val < 0 else val, eps_runtime_mean[i] - eps_cost_std[i]/2))
    # plt.fill_between(speeds_ratio, eps_runtime_mean[i] + eps_cost_std[i]/2, l, alpha=0.2)
    ax.plot(speeds_ratio, eps_cost_means[i], label=labels[i])
    # l = list(map(lambda val: 0 if val < 0 else val, eps_runtime_mean[i] - eps_cost_std[i] / 2))
    plt.fill_between(speeds_ratio, eps_cost_means[i] + eps_cost_std[i] * standard_error_const, eps_cost_means[i] - eps_cost_std[i] * standard_error_const, alpha=0.2)

ax.yaxis.set_major_formatter(mtick.PercentFormatter(xmax=1, decimals=None, symbol='%', is_latex=False))
plt.xlabel(r'$\frac{v_{\min}}{v_{\max}}$', size=30)
plt.ylabel(r'$\mathbf{Cost\,\,compare\,\,to\,\,}\textsf{T*}$')
plt.legend(prop={'size': 16})
plt.tight_layout(pad=0.05)
plt.savefig('Cost vs speeds ratio.pdf')
plt.show()

fig = plt.figure(1, (7, 4))
ax = fig.add_subplot(1, 1, 1)

for i in range(len(eps_cost_means)):
    # fig = plt.figure(1, (7, 4))
    # ax = fig.add_subplot(1, 1, 1)
    # ax.plot(speeds_ratio, eps_runtime_mean[i], label=labels[i])
    # l = list(map(lambda val: 0 if val < 0 else val, eps_runtime_mean[i] - eps_cost_std[i]/2))
    # plt.fill_between(speeds_ratio, eps_runtime_mean[i] + eps_cost_std[i]/2, l, alpha=0.2)
    ax.plot(speeds_ratio, eps_runtime_mean[i], label=labels[i])
    # l = list(map(lambda val: 0 if val < 0 else val, eps_runtime_mean[i] - eps_cost_std[i] / 2))
    plt.fill_between(speeds_ratio, eps_runtime_mean[i] + eps_runtime_std[i] / 2, eps_runtime_mean[i] - eps_runtime_std[i] / 2,
                     alpha=0.2)

ax.yaxis.set_major_formatter(mtick.PercentFormatter(xmax=1, decimals=None, symbol='%', is_latex=False))
plt.xlabel(r'$\frac{v_{\min}}{v_{\max}}$', size=30)
plt.ylabel(r'$\mathbf{Runtime\,\,compare\,\,to\,\,}\textsf{T*}$')
plt.legend(prop={'size': 16}, loc='upper left')
plt.tight_layout(pad=0.05)
plt.savefig('Runtime vs speeds ration.pdf')
plt.show()



# monte_carlo_read('new 1.txt', windx_02_windy_0_opt)

# wind, avarage = calc_lower_bound_quality('eps_4_wind_02_01.txt')
# a = 5
