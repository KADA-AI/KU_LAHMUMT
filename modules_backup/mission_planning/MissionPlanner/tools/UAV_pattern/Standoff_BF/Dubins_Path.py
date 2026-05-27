import numpy as np
import matplotlib.pyplot as plt

class DubinsPath:
    def __init__(self, R_min):
        self.R = R_min

    def rotz(self, theta):
        R = np.array([
            [np.cos(theta), -np.sin(theta), 0],
            [np.sin(theta),  np.cos(theta), 0],
            [0,              0,             1]
        ])
        return R

    def dubins_RLR(self, alpha, beta, d):
        tmp_rlr = (6. - d**2 + 2*np.cos(alpha - beta) + 2*d*(np.sin(alpha)-np.sin(beta))) / 8.
        if abs(tmp_rlr) > 1:
            return -1
        else:
            p = np.mod((2*np.pi - np.arccos(tmp_rlr)), 2*np.pi)
            t = np.mod((alpha - np.arctan2(np.cos(alpha)-np.cos(beta),
                                          d-np.sin(alpha)+np.sin(beta))
                         + np.mod(p/2, 2*np.pi)), 2*np.pi)
            q = np.mod((alpha - beta - t + np.mod(p, 2*np.pi)), 2*np.pi)
            return [t, p, q]

    def dubins_LRL(self, alpha, beta, d):
        tmp_lrl = (6. - d**2 + 2*np.cos(alpha - beta) + 2*d*(-np.sin(alpha) + np.sin(beta))) / 8.
        if abs(tmp_lrl) > 1:
            return -1
        else:
            p = np.mod((2*np.pi - np.arccos(tmp_lrl)), 2*np.pi)
            t = np.mod((-alpha - np.arctan2(np.cos(alpha)-np.cos(beta),
                                           d+np.sin(alpha)-np.sin(beta))
                         + p/2), 2*np.pi)
            q = np.mod((np.mod(beta, 2*np.pi) - alpha - t + np.mod(p, 2*np.pi)), 2*np.pi)
            return [t, p, q]

    def DubinsParamCal(self, start_node, end_node):
        R = self.R
        ell = np.linalg.norm(start_node[:2] - end_node[:2])
        ps   = start_node[:3]
        chis = start_node[3]
        pe   = end_node[:3]
        chie = end_node[3]

        crs = ps + R * self.rotz(np.pi/2).dot([np.cos(chis), np.sin(chis), 0])
        cls = ps + R * self.rotz(-np.pi/2).dot([np.cos(chis), np.sin(chis), 0])
        cre = pe + R * self.rotz(np.pi/2).dot([np.cos(chie), np.sin(chie), 0])
        cle = pe + R * self.rotz(-np.pi/2).dot([np.cos(chie), np.sin(chie), 0])

        # RSR
        ell = np.linalg.norm(crs - cre)
        theta = -(np.arctan2(cre[0]-crs[0], cre[1]-crs[1]) - np.pi/2)
        L1 = ell + R * np.mod(2*np.pi + np.mod(theta-np.pi/2,2*np.pi) - np.mod(chis-np.pi/2,2*np.pi),2*np.pi) \
           + R * np.mod(2*np.pi + np.mod(chie-np.pi/2,2*np.pi) - np.mod(theta-np.pi/2,2*np.pi),2*np.pi)

        # RSL
        ell = np.linalg.norm(cle - crs)
        theta = -(np.arctan2(cle[0]-crs[0], cle[1]-crs[1]) - np.pi/2)
        theta2 = theta - np.pi/2 + np.arcsin(2*R/ell)
        if not np.isreal(theta2):
            L2 = 9999999
        else:
            L2 = np.sqrt(ell**2 - 4*R**2) \
               + R * np.mod(2*np.pi + np.mod(theta2,2*np.pi) - np.mod(chis-np.pi/2,2*np.pi),2*np.pi) \
               + R * np.mod(2*np.pi + np.mod(theta2+np.pi,2*np.pi) - np.mod(chie+np.pi/2,2*np.pi),2*np.pi)

        # LSR
        ell = np.linalg.norm(cre - cls)
        theta = -(np.arctan2(cre[0]-cls[0], cre[1]-cls[1]) - np.pi/2)
        theta2 = np.arccos(2*R/ell)
        if not np.isreal(theta2):
            L3 = 9999999
        else:
            L3 = np.sqrt(ell**2 - 4*R**2) \
               + R * np.mod(2*np.pi + np.mod(chis+np.pi/2,2*np.pi) - np.mod(theta+theta2,2*np.pi),2*np.pi) \
               + R * np.mod(2*np.pi + np.mod(chie-np.pi/2,2*np.pi) - np.mod(theta+theta2-np.pi,2*np.pi),2*np.pi)

        # LSL
        ell = np.linalg.norm(cls - cle)
        theta = -(np.arctan2(cle[0]-cls[0], cle[1]-cls[1]) - np.pi/2)
        L4 = ell + R * np.mod(2*np.pi + np.mod(chis+np.pi/2,2*np.pi) - np.mod(theta+np.pi/2,2*np.pi),2*np.pi) \
           + R * np.mod(2*np.pi + np.mod(theta+np.pi/2,2*np.pi) - np.mod(chie+np.pi/2,2*np.pi),2*np.pi)

        # RLR
        inter_vec = cre - crs
        l = np.linalg.norm(inter_vec)
        if l > 4*R:
            L5 = 9999999
        else:
            tmp = R * inter_vec / l
            tmp_theta = np.arccos((l/2) / (2*R))
            rot_vec = self.rotz(tmp_theta).dot(tmp)
            cmid = 2 * rot_vec + crs
            theta1 = np.arctan2(cmid[1]-crs[1], cmid[0]-crs[0])
            theta2 = np.arctan2(cre[1]-cmid[1], cre[0]-cmid[0])
            L5 = R * np.mod(2*np.pi + np.mod(theta1,2*np.pi) - np.mod(chis-np.pi/2,2*np.pi),2*np.pi) \
               + R * np.mod(2*np.pi + np.mod(theta1+np.pi,2*np.pi) - np.mod(theta2,2*np.pi),2*np.pi) \
               + R * np.mod(2*np.pi + np.mod(chie-np.pi/2,2*np.pi) - np.mod(theta2+np.pi,2*np.pi),2*np.pi)

        # LRL
        inter_vec = cle - cls
        l = np.linalg.norm(inter_vec)
        if l > 4*R:
            L6 = 9999999
        else:
            tmp = R * inter_vec / l
            tmp_theta = np.arccos((l/2) / (2*R))
            rot_vec = self.rotz(-tmp_theta).dot(tmp)
            cmid = 2 * rot_vec + cls
            theta1 = np.arctan2(cmid[1]-cls[1], cmid[0]-cls[0])
            theta2 = np.arctan2(cle[1]-cmid[1], cle[0]-cmid[0])
            L6 = R * np.mod(2*np.pi + np.mod(chis+np.pi/2,2*np.pi) - np.mod(theta1,2*np.pi),2*np.pi) \
               + R * np.mod(2*np.pi + np.mod(theta2,2*np.pi) - np.mod(theta1+np.pi,2*np.pi),2*np.pi) \
               + R * np.mod(2*np.pi + np.mod(theta2+np.pi,2*np.pi) - np.mod(chie+np.pi/2,2*np.pi),2*np.pi)

        # 최소 거리 & 유형 선택
        L, idx = min((L1,1),(L2,2),(L3,3),(L4,4),(L5,5),(L6,6), key=lambda x: x[0])

        # 기하 파라미터 계산
        e1 = np.array([1,0,0])
        q1, q2 = None, None
        if idx == 1:
            cs, lams, ce, lame = crs, 1, cre, 1
            q1 = (ce - cs) / np.linalg.norm(ce - cs)
            w1 = cs + R * self.rotz(-np.pi/2).dot(q1)
            w2 = ce + R * self.rotz(-np.pi/2).dot(q1)
        elif idx == 2:
            cs, lams, ce, lame = crs, 1, cle, -1
            ell = np.linalg.norm(ce - cs)
            theta = -(np.arctan2(ce[0]-cs[0], ce[1]-cs[1]) - np.pi/2)
            theta2 = theta - np.pi/2 + np.arcsin(2*R/ell)
            q1 = self.rotz(theta2 + np.pi/2).dot(e1)
            w1 = cs + R * self.rotz(theta2).dot(e1)
            w2 = ce + R * self.rotz(theta2 + np.pi).dot(e1)
        elif idx == 3:
            cs, lams, ce, lame = cls, -1, cre, 1
            ell = np.linalg.norm(ce - cs)
            theta = -(np.arctan2(ce[0]-cs[0], ce[1]-cs[1]) - np.pi/2)
            theta2 = np.arccos(2*R/ell)
            q1 = self.rotz(theta + theta2 - np.pi/2).dot(e1)
            w1 = cs + R * self.rotz(theta + theta2).dot(e1)
            w2 = ce + R * self.rotz(theta + theta2 - np.pi).dot(e1)
        elif idx == 4:
            cs, lams, ce, lame = cls, -1, cle, -1
            q1 = (ce - cs) / np.linalg.norm(ce - cs)
            w1 = cs + R * self.rotz(np.pi/2).dot(q1)
            w2 = ce + R * self.rotz(np.pi/2).dot(q1)
        elif idx == 5:
            cs, lams, ce, lame = crs, 1, cre, 1
            inter_vec = ce - cs
            tmp = R * inter_vec / np.linalg.norm(inter_vec)
            l = np.linalg.norm(inter_vec)
            tmp_theta = np.arccos((l/2) / (2*R))
            rot_vec = self.rotz(tmp_theta).dot(tmp)
            cmid = 2 * rot_vec + cs
            lammid = -1
            w1 = (cs + cmid) / 2
            w2 = (ce + cmid) / 2
            tmp1 = np.mod(np.arctan2(cmid[1]-cs[1], cmid[0]-cs[0]) + np.pi/2, 2*np.pi)
            tmp2 = np.mod(np.arctan2(ce[1]-cmid[1], ce[0]-cmid[0]) - np.pi/2, 2*np.pi)
            q1 = np.array([np.cos(tmp1), np.sin(tmp1), 0])
            q2 = np.array([np.cos(tmp2), np.sin(tmp2), 0])
            q_tmp = (w1 + w2) / 2
            q2_tp = (q_tmp - cmid) / np.linalg.norm(q_tmp - cmid)
            w2_tmp = cmid + R * (-q2_tp)
            q2_tmp = self.rotz(-np.pi/2).dot(-q2_tp)
        else:  # idx == 6
            cs, lams, ce, lame = cls, -1, cle, -1
            inter_vec = ce - cs
            tmp = R * inter_vec / np.linalg.norm(inter_vec)
            l = np.linalg.norm(inter_vec)
            tmp_theta = np.arccos((l/2) / (2*R))
            rot_vec = self.rotz(-tmp_theta).dot(tmp)
            cmid = 2 * rot_vec + cs
            lammid = 1
            w1 = (cs + cmid) / 2
            w2 = (ce + cmid) / 2
            tmp1 = np.mod(np.arctan2(cmid[1]-cs[1], cmid[0]-cs[0]) - np.pi/2, 2*np.pi)
            tmp2 = np.mod(np.arctan2(ce[1]-cmid[1], ce[0]-cmid[0]) + np.pi/2, 2*np.pi)
            q1 = np.array([np.cos(tmp1), np.sin(tmp1), 0])
            q2 = np.array([np.cos(tmp2), np.sin(tmp2), 0])
            q_tmp = (w1 + w2) / 2
            q2_tp = (q_tmp - cmid) / np.linalg.norm(q_tmp - cmid)
            w2_tmp = cmid + R * (-q2_tp)
            q2_tmp = self.rotz(np.pi/2).dot(-q2_tp)

        w3 = pe
        q3 = self.rotz(chie).dot(e1)

        dubinspath = {
            'ps': ps, 'chis': chis, 'pe': pe, 'chie': chie, 'R': R, 'L': L,
            'cs': cs, 'lams': lams, 'ce': ce, 'lame': lame,
            'w1': w1, 'w2': w2, 'w3': w3, 'q3': q3, 'idx': idx
        }
        if idx in [5, 6]:
            dubinspath.update({
                'cmid': cmid, 'lammid': lammid,
                'w2_tmp': w2_tmp, 'q2_tmp': q2_tmp,
                'q1': q1, 'q2': q2
            })
        else:
            dubinspath.update({'q1': q1, 'q2': q2})
        return dubinspath

    
    def plan(self, nodes):
        path = []
        for i in range(len(nodes) - 1):
            path.append(self.DubinsParamCal(nodes[i], nodes[i+1]))
        return path

    
    def plot(self, nodes):
        num_nodes = len(nodes)
        for i in range(num_nodes - 1):
            start, end = nodes[i], nodes[i+1]
            dp = self.DubinsParamCal(start, end)
            if dp['idx'] < 5:
                plt.plot([dp['w2'][1], dp['w1'][1]],
                         [dp['w2'][0], dp['w1'][0]], 'g-', linewidth=2)
                s1 = np.mod(2*np.pi + np.arctan2(start[0]-dp['cs'][0],
                                                 start[1]-dp['cs'][1]), 2*np.pi)
                s2 = np.mod(2*np.pi + np.arctan2(dp['w1'][0]-dp['cs'][0],
                                                 dp['w1'][1]-dp['cs'][1]), 2*np.pi)
                s_angle = np.linspace(s2, s1, 80)
                if (dp['lams']==1 and s2> s1) or (dp['lams']==-1 and s1> s2):
                    max_s, min_s = max(s1, s2), min(s1, s2)
                    temp = np.hstack([np.linspace(0, min_s, 40),
                                      np.linspace(0, max_s-2*np.pi, 40)])
                    s_angle = np.sort(temp)
                plt.plot(self.R*np.cos(s_angle)+dp['cs'][1],
                         self.R*np.sin(s_angle)+dp['cs'][0], 'g-', linewidth=2)

                e1 = np.mod(2*np.pi + np.arctan2(dp['w2'][0]-dp['ce'][0],
                                                 dp['w2'][1]-dp['ce'][1]), 2*np.pi)
                e2 = np.mod(2*np.pi + np.arctan2(end[0]-dp['ce'][0],
                                                 end[1]-dp['ce'][1]), 2*np.pi)
                e_angle = np.linspace(e2, e1, 80)
                if (dp['lame']==1 and e2> e1) or (dp['lame']==-1 and e1> e2):
                    max_e, min_e = max(e1, e2), min(e1, e2)
                    temp = np.hstack([np.linspace(0, min_e, 40),
                                      np.linspace(0, max_e-2*np.pi, 40)])
                    e_angle = np.sort(temp)
                plt.plot(self.R*np.cos(e_angle)+dp['ce'][1],
                         self.R*np.sin(e_angle)+dp['ce'][0], 'g-', linewidth=2)

                plt.quiver(dp['ps'][1], dp['ps'][0],
                           np.cos(np.pi/2-dp['chis']),
                           np.sin(np.pi/2-dp['chis']),
                           scale=30, color=[0,0,1], linewidth=3)
                plt.quiver(dp['pe'][1], dp['pe'][0],
                           np.cos(np.pi/2-dp['chie']),
                           np.sin(np.pi/2-dp['chie']),
                           scale=30, color=[0,0,1], linewidth=3)
            else:
                # 중간 arc
                s1 = np.mod(2*np.pi + np.arctan2(start[0]-dp['cs'][0],
                                                 start[1]-dp['cs'][1]), 2*np.pi)
                s2 = np.mod(2*np.pi + np.arctan2(dp['w1'][0]-dp['cs'][0],
                                                 dp['w1'][1]-dp['cs'][1]), 2*np.pi)
                s_angle = np.linspace(s2, s1, 80)
                if (dp['lams']==1 and s2> s1) or (dp['lams']==-1 and s1> s2):
                    max_s, min_s = max(s1, s2), min(s1, s2)
                    temp = np.hstack([np.linspace(0, min_s, 40),
                                      np.linspace(0, max_s-2*np.pi, 40)])
                    s_angle = np.sort(temp)
                plt.plot(self.R*np.cos(s_angle)+dp['cs'][1],
                         self.R*np.sin(s_angle)+dp['cs'][0], 'g-', linewidth=2)

                # 끝 arc
                e1 = np.mod(2*np.pi + np.arctan2(dp['w2'][0]-dp['ce'][0],
                                                 dp['w2'][1]-dp['ce'][1]), 2*np.pi)
                e2 = np.mod(2*np.pi + np.arctan2(end[0]-dp['ce'][0],
                                                 end[1]-dp['ce'][1]), 2*np.pi)
                e_angle = np.linspace(e2, e1, 80)
                if (dp['lame']==1 and e2> e1) or (dp['lame']==-1 and e1> e2):
                    max_e, min_e = max(e1, e2), min(e1, e2)
                    temp = np.hstack([np.linspace(0, min_e, 40),
                                      np.linspace(0, max_e-2*np.pi, 40)])
                    e_angle = np.sort(temp)
                plt.plot(self.R*np.cos(e_angle)+dp['ce'][1],
                         self.R*np.sin(e_angle)+dp['ce'][0], 'g-', linewidth=2)

                # 중간 arc (돌아오는 부분)
                m1 = np.mod(2*np.pi + np.arctan2(dp['w1'][0]-dp['cmid'][0],
                                                 dp['w1'][1]-dp['cmid'][1]), 2*np.pi)
                m2 = np.mod(2*np.pi + np.arctan2(dp['w2'][0]-dp['cmid'][0],
                                                 dp['w2'][1]-dp['cmid'][1]), 2*np.pi)
                m_angle = np.linspace(m2, m1, 80)
                if (dp['lammid']==1 and m2> m1) or (dp['lammid']==-1 and m1> m2):
                    max_m, min_m = max(m1, m2), min(m1, m2)
                    temp = np.hstack([np.linspace(0, min_m, 40),
                                      np.linspace(0, max_m-2*np.pi, 40)])
                    m_angle = np.sort(temp)
                plt.plot(self.R*np.cos(m_angle)+dp['cmid'][1],
                         self.R*np.sin(m_angle)+dp['cmid'][0], 'g-', linewidth=2)

                plt.quiver(dp['ps'][1], dp['ps'][0],
                           np.cos(np.pi/2-dp['chis']),
                           np.sin(np.pi/2-dp['chis']),
                           scale=30, color=[0,0,1], linewidth=3)
                plt.quiver(dp['pe'][1], dp['pe'][0],
                           np.cos(np.pi/2-dp['chie']),
                           np.sin(np.pi/2-dp['chie']),
                           scale=30, color=[0,0,1], linewidth=3)
        plt.show()

    def nodes_with_tangents(self, nodes):
        """
        nodes: list of [x,y,z,heading]
        반환: 입력된 노드 (x,y) 와 
              각 세그먼트의 w1, w2 (tangent points)를 
              순서대로 끼워 넣은 [[x,y], …] 배열
        """
        pts = []
        for i in range(len(nodes)-1):
            # 1) 입력 노드
            x0, y0 = nodes[i][0], nodes[i][1]
            pts.append([x0, y0])

            # 2) Dubins 파라미터 계산
            dp = self.DubinsParamCal(nodes[i], nodes[i+1])

            # 3) w1, w2 → [x,y] 순서로
            w1_x, w1_y = dp['w1'][0], dp['w1'][1]
            w2_x, w2_y = dp['w2'][0], dp['w2'][1]
            pts.append([w1_x, w1_y])
            pts.append([w2_x, w2_y])

        # 마지막 노드
        xN, yN = nodes[-1][0], nodes[-1][1]
        pts.append([xN, yN])

        return np.array(pts)

    def cost(self, wplist_order):
        wplist_order = np.array(wplist_order)
        wp_num = wplist_order.shape[0]
        wp_num_half = 0.5 * wp_num
        total_length = 0
        if wp_num_half == 1:
            total_length = np.linalg.norm(wplist_order[0] - wplist_order[1])
        else:
            for iw in range(wp_num - 1):
                if iw % 2 == 1:
                    total_length += np.linalg.norm(wplist_order[iw] - wplist_order[iw + 1])
                else:
                    param = self.DubinsParamCal(wplist_order[iw], wplist_order[iw + 1])
                    total_length += param['L']
        return total_length

    def get_branch_points(self, nodes):
        """ 각 분기점 반환"""
        pts = []
        for i in range(len(nodes) - 1):
            dp = self.DubinsParamCal(nodes[i], nodes[i+1])
            # w1, w2의 앞 두 요소(x,y)만 취한다
            pts.append(dp['w1'][:2])
            pts.append(dp['w2'][:2])
        return np.array(pts)




# nodes = [
#     # np.array([0.0, 100, 0, 0]),
#     np.array([900.0, 100, 610, 0]),
#     np.array([900.0, 500, 610, np.pi]),
#     # np.array([0.0, 500, 0, np.pi]),
# ]
# dubins = DubinsPath(R_min=360)
# path_params = dubins.plan(nodes)      # 최종 path 파라미터 반환
# all_pts = dubins.nodes_with_tangents(nodes)
# # # print(all_pts)
# # print(path_params)
# dubins.plot(nodes)                    # 경로 시각화
# # length = dubins.cost(nodes)           # 전체 경로 길이 계산
# # dubins = DubinsPath(R_min=100)
# branch_pts = dubins.get_branch_points(nodes)
# print(branch_pts)
