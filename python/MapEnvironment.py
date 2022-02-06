import numpy as np
from IPython import embed
from matplotlib import pyplot as plt
import matplotlib
matplotlib.rcParams['pdf.fonttype'] = 42
matplotlib.rcParams['ps.fonttype'] = 42


class MapEnvironment(object):
    
    def __init__(self, mapfile, start, goal):

        # Obtain the boundary limits.
        # Check if file exists.
        self.map = np.flipud(np.loadtxt(mapfile))
        self.xlimit = [0, np.shape(self.map)[0]-1] # TODO (avk): Check if this needs to flip.
        self.ylimit = [0, np.shape(self.map)[1]-1]
        
        # Check if start and goal are within limits and collision free
        if not self.state_validity_checker(start) or not self.state_validity_checker(goal):
            raise ValueError('Start and Goal state must be within the map limits');
            exit(0)
        # store start and goal
        self.start = start
        self.goal = goal
        # Display the map
        #plt.imshow(self.map, interpolation='nearest')
        #plt.show()
        
        
    def compute_distance(self, start_config, end_config):
        s, g = np.array(start_config), np.array(end_config)
        return np.linalg.norm(g-s)

    def state_validity_checker(self, config):
        # check within limit
        if config[0] < self.xlimit[0] or config[0] > self.xlimit[1] or\
         config[1] < self.ylimit[0] or config[1] > self.ylimit[1]:
            return False
        # check collision
        x, y = int(config[0]), int(config[1])
        return not self.map[x][y]

    def to_map(self, config):
        ''' helper function mapping continuous config to discredited map'''
        return config.astype(int)

    def edge_validity_checker(self, config1, config2, step_size=0.3):
        if not self.state_validity_checker(config1) or not self.state_validity_checker(config2):
            return False
        c1, c2 = np.array(config1).astype(float)+.5, np.array(config2).astype(float)+.5
        v_norm = np.linalg.norm(c2-c1)
        count = 1
        while (count * step_size) < v_norm:
            a = count * step_size / v_norm
            check_config = (1-a)*c1 + a*c2
            if not self.state_validity_checker(check_config):
                return False
            count += 1
        return True

    def compute_heuristic(self, config):
        return self.compute_distance(config, self.goal)

    def visualize_plan(self, plan, wind, cells_path, plot_both_dubins=False):
        '''
        Visualize the final path
        @param plan Sequence of states defining the plan.
        '''
        # Remove white spaces
        figsize = (14, 14) # 14 x 14 inch plot
        fig, ax = plt.subplots(figsize=figsize)
        fig.tight_layout()
        fig.subplots_adjust(left=0, bottom=0, right=1, top=1, wspace=None, hspace=None)
        ax.imshow(self.map, cmap='Greys', interpolation='nearest')

        # # Original
        # plt.imshow(self.map, cmap='Greys', interpolation='nearest')
        # we flip back the solution and the map 
        plt.gca().invert_yaxis()

        # Plot arrows for orientations
        orient = 8
        r = 0.7
        for i, item in enumerate(cells_path):
            # if i != 0 and i != (len(cells_path) - 1):  # plot orientation only for start and goal
            #     continue
            theta = item[2] * 2 * np.pi / orient
            plt.arrow(item[1], item[0], r * np.cos(theta), r * np.sin(theta), color='#3897C5', head_width=.25, width=0.05)  # teal color for T*
            # plt.arrow(item[1], item[0], r * np.cos(theta), r * np.sin(theta), color='#973F8A', head_width=.25, width=0.05)  # purple for minimum speed Dubins
            # plt.arrow(item[1], item[0], r * np.cos(theta), r * np.sin(theta), color='#E88223', head_width=.25, width=0.05)  # orange color for T*epsilon

        # Plot wind
        windx = wind[0]
        windy = wind[1]
        if windy or windx:  # Plot only if there is wind
            for x in range(1, self.xlimit[1] - 1, 2):
                for y in range(1, self.ylimit[1], 2):
                    if (x < 5) and y == 11:
                        continue
                    if x < 5 and y == 1:
                        continue
                    plt.arrow(x, y, windx * 5.5, windy * 5.5, color='b', head_width=.35, width=0.05, alpha=0.6)

        start_goal_font = 45

        plt.plot(self.start[1], self.start[0], 'o', color='r', markersize=40)
        plt.annotate('Start', xy=(self.start[1] + 0.75, self.start[0] + 0), color='purple', fontsize=start_goal_font)
        # plt.annotate('Start', xy=(self.start[1] + 1.2, self.start[0] - 0.25), color='r', fontsize=start_goal_font)

        # plt.plot(4, 11, 'o', color='g', markersize=40)
        # plt.annotate('Goal', xy=(2.5, 12.5), color='purple', fontsize=start_goal_font)

        plt.plot(self.goal[1], self.goal[0], 'o', color='g', markersize=40)
        plt.annotate('Goal', xy=(self.goal[1] - 1.5, self.goal[0] + 1.5), color='purple', fontsize=start_goal_font)
        # plt.annotate('Goal', xy=(self.goal[1] + -.5, self.goal[0] + 1), color='g', fontsize=start_goal_font)
        for i in range(np.shape(plan)[0] - 1):
            x = [plan[i,0], plan[i+1, 0]]
            y = [plan[i,1], plan[i+1, 1]]
            if plot_both_dubins and int(plan[i,3]) != int(plan[i + 1,3]):
                continue
            if int(plan[i, 3]) == 0:  # slow speed
                # plt.plot(y, x, 'r', linewidth=5)
                plt.plot(y, x, 'r', linewidth=5, alpha=0.35)
            elif int(plan[i, 3]) == 1:  # high speed
                # plt.plot(y, x, 'g', linewidth=5)
                plt.plot(y, x, 'g', linewidth=5, alpha=0.35)
        # plt.gca().set_xticks(np.arange(0, self.xlimit[1]+1, 5), minor=False)
        plt.gca().set_xticks(np.arange(0.5, self.xlimit[1]+1, 1), minor=True)
        # plt.gca().set_xticks([])

        plt.gca().xaxis.grid(False, which='major')
        plt.gca().xaxis.grid(True, which='minor')
        # plt.gca().set_yticks(np.arange(0, self.ylimit[1]+1, 1), minor=False)
        plt.gca().set_yticks(np.arange(0.5, self.ylimit[1]+1, 1), minor=True)
        # plt.gca().set_yticks([])
        plt.gca().yaxis.grid(False, which='major')
        plt.gca().yaxis.grid(True, which='minor')

        # Set axes values fontsize

        plt.gca().tick_params(axis='both', labelsize=36)

        plt.tight_layout(pad=.05)
        plt.savefig('map10_maximum_speed_dubins.svg', transparent=True)
        plt.show()

    def visualize_env(self):
        '''
        Visualize the environment
        '''
        plt.imshow(self.map, interpolation='nearest')
        plt.plot(self.start[1], self.start[0], 'o', color='r')
        plt.plot(self.goal[1], self.goal[0], 'o', color='g')
        plt.show()