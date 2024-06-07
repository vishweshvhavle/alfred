from simple_pid import PID

class Controller:

    def __init__(self, k_v, k_w) -> None:
        kv_p, kv_i, kv_d = k_v
        kw_p, kw_i, kw_d = k_w
        self.CONTROLLER_V = PID(kv_p, kv_i, kv_d, setpoint=0)
        self.CONTROLLER_W = PID(kw_p, kw_i, kw_d, setpoint=0)

        self.CONTROLLER_V.output_limits = (0 , 1)
        self.CONTROLLER_W.output_limits = (-1, 1)

    def get_action(self, current_state):
        distance = -current_state[0]
        theta = current_state[1]

        velocity = self.CONTROLLER_V(distance)
        omega = self.CONTROLLER_W(theta)

        return velocity, omega


    

