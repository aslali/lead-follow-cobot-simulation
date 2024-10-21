import matplotlib.pyplot as plt
import matplotlib
import time
import pickle
import os


class Measure:

    def __init__(self, case_name, fast_run=False, rfast=1):
        self.action_times_human = []
        self.action_times_robot = []
        self.total_times_human = 0
        self.total_times_robot = 0
        self.robot_travel_distance = []
        self.human_travel_distance = []
        self.p_f = []
        self.p_e = []
        self.de = {}
        self.df = {}
        self.init_time = None
        self.case_name = case_name
        self.twrong = None
        self.nwrong = None
        self.n_tot_hum_assign = None
        self.n_error2 = None
        self.n_self_hum_assign = None
        self.n_tot_rob_assign = None
        self.n_self_rob_assign = None
        self.idle_time = None
        self.rob_time = None
        self.hum_time = None
        self.sim_update = not fast_run
        if fast_run:
            self.rfast = rfast
        else:
            self.rfast = 1

    def start_time(self):
        tic = time.perf_counter()
        return tic

    def action_end(self, start_time_total, agent, start_time_action=None, idle_time=None, travel_distance=None,
                   action_type=None, action_number=None):
        toc = time.perf_counter()
        if agent == 'human':
            action_time = toc - start_time_total
            self.action_times_human.append(
                ((start_time_total - self.init_time) * self.rfast, action_time * self.rfast, idle_time * self.rfast,
                 action_type, action_number))

            self.total_times_human += action_time
            self.human_travel_distance.append(travel_distance)
        else:
            planning_time = start_time_action - start_time_total
            action_time = toc - start_time_action
            plan_action_time = toc - start_time_total
            self.total_times_robot += plan_action_time
            self.action_times_robot.append(
                ((start_time_total - self.init_time) * self.rfast, (start_time_action - self.init_time) * self.rfast,
                 planning_time * self.rfast, action_time * self.rfast, plan_action_time * self.rfast, action_type,
                 action_number))
            self.robot_travel_distance.append(travel_distance)

    def creat_table(self):
        wrong = [x[3] for x in self.action_times_human if (x[3] == 'error1' or x[3] == 'error2')]
        rwrong = [x[4] for x in self.action_times_robot if (x[5] == 'error1' or x[5] == 'error2')]
        self.twrong = sum(rwrong)
        self.nwrong = len(wrong)
        print('n wrong actions: ', self.nwrong)
        print('t wrong actions: ', self.twrong)

        hassign = [x[3] for x in self.action_times_human if x[3] == 'tray2']
        hhassign = [x[3] for x in self.action_times_human if x[3] == 'allocate' or x[3] == 'error2']
        hhhassign = [x[3] for x in self.action_times_human if x[3] == 'error2']
        self.n_tot_hum_assign = len(hhassign)
        self.n_error2 = len(hhhassign)
        self.n_self_hum_assign = len(hassign)
        print('n assigned by human: ', self.n_self_hum_assign, ' -- ', self.n_error2)
        print('n assigned total by human: ', self.n_tot_hum_assign)

        rassign = [x[3] for x in self.action_times_robot if x[5] == 'allocate']
        rassign2 = [x[3] for x in self.action_times_robot if x[5] == 'tray2']

        self.n_tot_rob_assign = len(rassign)
        self.n_self_rob_assign = len(rassign2)
        print('n assigned by robot: ', self.n_tot_rob_assign, ' -- ', self.n_self_rob_assign)

        idletime = [x[2] for x in self.action_times_human]
        self.idle_time = sum(idletime)
        print('t idle: ', self.idle_time)

        tr = [x[4] for x in self.action_times_robot]
        self.rob_time = sum(tr)
        print('t total robot: ', self.rob_time)

        th = [x[1] for x in self.action_times_human]
        self.hum_time = sum(th)
        print('t total human: ', self.hum_time)

        self.dr = sum(self.robot_travel_distance)
        self.dh = sum(self.human_travel_distance)
        print('d total robot: ', self.dr)
        print('d total human: ', self.dh)

    def human_measures(self, start_time, p_following, p_error):
        self.p_f.append((start_time - self.init_time, p_following))
        self.p_e.append((start_time - self.init_time, p_error))
        print('time: ', start_time -self.init_time, ' measure')
        # self.plot_human_measures()


    def plot_human_measures(self):
        fig, ax = plt.subplots()
        x_val1 = [x[0] * self.rfast for x in self.p_f]
        y_val1 = [x[1] for x in self.p_f]
        x_val2 = [x[0] * self.rfast for x in self.p_e]
        y_val2 = [x[1] for x in self.p_e]
        ax.plot(x_val1, y_val1, linewidth=3)
        ax.plot(x_val2, y_val2, linewidth=3)
        ax.set_xlabel('time (s)', fontsize=16)
        ax.set_ylabel(r'$\alpha_e, \alpha_f$', fontsize=16)
        # lgd=ax.legend([r'$\alpha_f$', r'$\alpha_e$'], fontsize=15, loc='upper left', frameon=False,
        #           bbox_to_anchor=(0.25, -0.1), ncol=2)
        lgd = ax.legend([r'$\alpha_f$: Following Preference', r'$\alpha_p$: Error-proneness'],
                        fontsize=12, loc='upper left', frameon=False,
                        ncol=1)
        bbox_to_anchor=(0.25, 1)
        ax.set_title(r'Bayes estimates of $\alpha_e$ and $\alpha_f$', fontsize=16)
        ax.set_ylim([0, 1.1])
        ax.set_xlim([0, round(x_val1[-1]+10)])
        ax.set_xticks(range(0, round(x_val1[-1]+10), 20))
        ax.set_yticks([0.0, 0.2, 0.4, 0.6, 0.8, 1.0])
        ax.tick_params(axis='x', labelsize=16)
        ax.tick_params(axis='y', labelsize=16)
        plt.tight_layout()
        fig.savefig('simul_estimate98.eps', format='eps')
        plt.show()

    def plot_human_measure_ind(self):
        for i in range(len(self.p_f)):
            fig, ax = plt.subplots()
            x_val1 = [x[0] * self.rfast for x in self.p_f[0:i+1]]
            y_val1 = [x[1] for x in self.p_f[0:i+1]]
            x_val2 = [x[0] * self.rfast for x in self.p_e[0:i+1]]
            y_val2 = [x[1] for x in self.p_e[0:i+1]]
            ax.plot(x_val1, y_val1, linewidth=3)
            ax.plot(x_val2, y_val2, linewidth=3)
            ax.set_xlabel('time (s)', fontsize=16)
            ax.set_ylabel(r'$p_e, p_f$')
            ax.set_title(r'Expected values of $p_e$ and $p_f$', fontsize=16)
            ax.set_ylim([0, 1])
            ax.set_xlim([0, 140])
            ax.set_xticks([0, 20, 40, 60, 80, 100, 120, 140])
            ax.set_yticks([0.0, 0.2, 0.4, 0.6, 0.8, 1.0])
            ax.tick_params(axis='x', labelsize=16)
            ax.tick_params(axis='y', labelsize=16)
            filename = 'videoim2/' + 'mm' + str(round(x_val1[-1], 2)) + '.png'
            # plt.savefig(filename, format='png', bbox_inches='tight', pad_inches=0.1)

    def human_dist_error(self, start_time, pe, se):
        st = (start_time - self.init_time) * self.rfast
        self.de[st] = {'perror': pe, 'eset': se}
        print('time: ', st, ' error')

    def plot_dists_error_ind(self):
        for i in self.de:
            se = self.de[i]['eset']
            pe = self.de[i]['perror']
            fig, ax = plt.subplots()
            ax.plot(se, pe, linewidth=3)
            ax.set_ylim([0, 1])
            ax.set_xlim([0, 1])
            ax.set_title('Belief about human error', fontsize=16)
            ax.set_xlabel(r'$p_e$', fontsize=16)
            ax.set_ylabel(r'$P(p_e)$', fontsize=16)
            ax.set_xticks([0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0])
            ax.tick_params(axis='x', labelsize=16)
            ax.tick_params(axis='y', labelsize=16)
            filename = 'videoim/' + 'ee' + str(round(i, 2)) + '.png'
            # plt.savefig(filename, format='png', bbox_inches='tight', pad_inches=0.1)

    def plot_dists_error(self):
        nd = len(self.de)
        nc = 3
        nr = nd // nc
        if nd % nc > 0:
            nr += 1
        fig, axs = plt.subplots(nr, nc, squeeze=False)
        # fig.tight_layout()

        i = 0
        j = 0
        for p in self.de:
            axs[i, j].plot(self.de[p]['eset'], self.de[p]['perror'])
            plt.subplots_adjust(hspace=0.2, wspace=0.2)
            axs[i, j].set_xlim([0, 1])
            axs[i, j].set_ylim([0, 1])
            tit = 't={}'.format(round(p, 2))
            axs[i, j].set_title(tit, y=1.0, pad=-14, fontsize=13)
            if j > 0:
                axs[i, j].set_yticklabels([])
            elif i == 1:
                pass
                # axs[i, j].set_ylabel(r'${P}(p_e)$', fontsize=15)

            if i == nr - 1:
                if j == 1:
                    pass
                    # axs[i, j].set_xlabel(r'$p_e$', fontsize=15)

            else:
                axs[i, j].set_xticklabels([])

            j += 1
            if j == nc:
                j = 0
                i += 1
        aa = nc - nd % nc
        if aa != nc:
            for ii in range(aa):
                fig.delaxes(axs[i, j + ii])

        fig.text(0.5, 0.04, r'$\alpha_e$', fontsize=15, ha='center')
        fig.text(0.04, 0.5, r'$P(\alpha_e)$', fontsize=15, va='center', rotation='vertical')
        # plt.savefig('dist_error5.eps', format='eps', bbox_inches='tight', pad_inches=0)
        plt.show()

    def human_dist_follow(self, start_time, pf, sf):
        st = (start_time - self.init_time) * self.rfast
        self.df[st] = {'pfollow': pf, 'fset': sf}

        # plt.show()
        print('time: ', st, ' follow')

    def plot_dists_follow_ind(self):
        for i in self.df:
            sf = self.df[i]['fset']
            pf = self.df[i]['pfollow']
            fig, ax = plt.subplots()
            ax.plot(sf, pf, linewidth=3)
            ax.set_ylim([0, 1])
            ax.set_xlim([0, 1])
            ax.set_title('Belief about preference', fontsize=16)
            ax.set_xlabel(r'$p_f$', fontsize=16)
            ax.set_ylabel(r'$P(p_f)$', fontsize=16)
            ax.set_xticks([0.0, 0.2, 0.4, 0.6, 0.8, 1.0])
            ax.tick_params(axis='x', labelsize=16)
            ax.tick_params(axis='y', labelsize=16)
            filename = 'videoim/' + 'ff' + str(round(i, 2)) + '.png'
            # plt.savefig(filename, format='png', bbox_inches='tight', pad_inches=0.1)
    #
    def plot_dists_follow(self):
        nd = len(self.df)
        nc = 3
        nr = nd // nc
        if nd % nc > 0:
            nr += 1
        fig, axs = plt.subplots(nr, nc, squeeze=False)

        i = 0
        j = 0
        for p in self.df:
            axs[i, j].plot(self.df[p]['fset'], self.df[p]['pfollow'])
            plt.subplots_adjust(hspace=0.2, wspace=0.2)
            axs[i, j].set_xlim([0, 1])
            axs[i, j].set_ylim([0, 1])
            tit = 't={}'.format(round(p, 2))
            axs[i, j].set_title(tit, y=1.0, pad=-14, fontsize=13)
            if j > 0:
                axs[i, j].set_yticklabels([])
            elif i == 1:
                pass
                # axs[i, j].set_ylabel(r'$P(p_f)$', fontsize=15)

            if i == nr - 1:
                if j == 1:
                    pass
                    # axs[i, j].set_xlabel(r'$p_f$', fontsize=15)
            else:
                axs[i, j].set_xticklabels([])

            j += 1
            if j == nc:
                j = 0
                i += 1

        aa = nc - nd % nc
        if aa != nc:
            for ii in range(aa):
                fig.delaxes(axs[i, j + ii])

        fig.text(0.5, 0.04, r'$\alpha_f$', fontsize=15, ha='center')
        fig.text(0.04, 0.5, r'$P(\alpha_f)$', fontsize=15, va='center', rotation='vertical')
        # plt.savefig('dist_follow_4.eps', format='eps', bbox_inches='tight', pad_inches=0)
        plt.show()

    def plot_times_actions(self):
        htime_gantt = []
        rtime_gantt = []
        htime_colorface = []
        rtime_colorface = []
        col = {'error1': 'tab:red', 'error2': '#D25E5D', 'allocate': 'tab:blue', 'tray1': '#6BF3FC',
               'normal': 'tab:green', 'tray2': '#27F727', 'idle': '#e7edf3'}
        for ii in self.action_times_human:
            htime_gantt.append((ii[0], ii[1]))
            htime_colorface.append(col[ii[3]])

        for ii in self.action_times_robot:
            rtime_gantt.append((ii[0], ii[4]))
            rtime_colorface.append(col[ii[5]])

        fig, ax = plt.subplots()
        prop = {'edgecolor': 'k'}
        ax.broken_barh(rtime_gantt, (10, 9), facecolors=rtime_colorface, **prop)
        ax.broken_barh(htime_gantt, (20, 9), facecolors=htime_colorface, **prop)
        ax.set_ylim(5, 35)
        ax.set_xlim(0, 200)
        ax.set_xlabel('Time (s)')
        ax.set_yticks([15, 25], labels=['Robot', 'Human'])
        ax.grid(True)
        plt.show()

    def run_all(self):

        self.creat_table()
        filename = self.case_name + ".pickle"
        try:
            os.makedirs(os.path.dirname(filename), exist_ok=True)
            with open(filename, "wb") as f:
                pickle.dump(self, f, protocol=pickle.HIGHEST_PROTOCOL)
        except Exception as ex:
            print("Error during pickling object (Possibly unsupported):", ex)

        self.plot_times_actions()
        self.plot_human_measures()
        self.plot_dists_error()
        self.plot_dists_follow()
        # self.plot_human_measure_ind()
        # self.plot_dists_follow_ind()
        # self.plot_dists_error_ind()

